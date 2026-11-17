//! PointCloud2 to compact zenoh-gateway fields (zstd-compressed by the bridge by default): optional
//! thinning to a point budget (a voxel grid), xyz quantized to int16 around a per-message origin, optional u8
//! intensity. Fields, see README "Wire formats": `version=3`, `count`, `sourceCount`, `origin` (f32 × 3),
//! `scale`, `keepEvery`, `maxError`, `positions` (scaled i16 x, y, z), `intensity` (u8, if the cloud
//! has it), `intensityMin`, `intensityScale`.
//!
//! Decoded `x = originX + qx * scale`. The error per axis of each sent point is at most `scale / 2`,
//! where `scale = (largest bounding-box extent / 2) / 32767` (plus f32 rounding, ~1e-7 relative), in
//! whatever units the cloud uses. Thinning at quality q keeps about q of the points (at least 1 in
//! MAX_KEEP_EVERY): the first point of each cell of a voxel grid sized to that budget, in message order, so
//! it never moves a point and spreads what it keeps over the cloud's space (1-in-N in message order kept
//! every Nth point of each scan line: dense near the robot, gaps far away, and whole rings dropped from a
//! ring-ordered lidar). `keepEvery` is the source count over the kept count, rounded.

use crate::codecs::msgs::{Header, PointCloud, PointField};
use anyhow::{Context, Result, bail, ensure};
use std::borrow::Cow;
use zenoh_gateway::Fields;

pub const VERSION: u8 = 3;
/// Thinning at quality 0: keep 1 point in this many.
pub const MAX_KEEP_EVERY: u32 = 16;
const QUANT_MAX: f64 = 32767.0;

/// Keep 1 point in N, N = round(1 / quality): 1 at quality 1, 2 at 0.5, 3 at 0.33, MAX_KEEP_EVERY at 0.
pub fn keep_every(quality: f64) -> u32 {
    (1.0 / quality.clamp(1.0 / MAX_KEEP_EVERY as f64, 1.0)).round() as u32
}

/// Reads one numeric field of a point as f64, for any PointField datatype.
#[derive(Clone, Copy)]
struct FieldReader {
    offset: usize,
    datatype: u8,
    big_endian: bool,
}

impl FieldReader {
    fn new(field: &PointField, point_step: u32, big_endian: bool) -> Result<Self> {
        let size = match field.datatype {
            1 | 2 => 1,
            3 | 4 => 2,
            5..=7 => 4,
            8 => 8,
            other => bail!("PointField {:?} has unknown datatype {other}", field.name),
        };
        ensure!(field.offset as usize + size <= point_step as usize, "PointField {:?} (offset {}) outside point_step {point_step}", field.name, field.offset);
        Ok(FieldReader { offset: field.offset as usize, datatype: field.datatype, big_endian })
    }

    fn read(&self, point: &[u8]) -> f64 {
        let bytes = &point[self.offset..];
        macro_rules! num {
            ($ty:ty, $n:expr) => {{
                let array: [u8; $n] = bytes[..$n].try_into().unwrap();
                (if self.big_endian { <$ty>::from_be_bytes(array) } else { <$ty>::from_le_bytes(array) }) as f64
            }};
        }
        match self.datatype {
            1 => bytes[0] as i8 as f64,
            2 => bytes[0] as f64,
            3 => num!(i16, 2),
            4 => num!(u16, 2),
            5 => num!(i32, 4),
            6 => num!(u32, 4),
            7 => num!(f32, 4),
            _ => num!(f64, 8),
        }
    }
}

#[derive(Clone)]
struct Point {
    xyz: [f64; 3],
    intensity: f64,
}

/// A cloud's finite points, owned (the decoded frame shared across frontends).
pub struct Points {
    points: Vec<Point>,
    has_intensity: bool,
    /// the cloud's header, sent with it: one topic can carry clouds in different frames
    header: Header,
}

/// Every finite point of the cloud, honoring offsets, point_step, row_step and endianness.
pub fn read_points(cloud: &PointCloud) -> Result<Points> {
    let field = |name: &str| cloud.fields.iter().find(|field| field.name == name);
    let reader = |name: &str| -> Result<FieldReader> {
        FieldReader::new(field(name).with_context(|| format!("PointCloud2 has no {name:?} field"))?, cloud.point_step, cloud.big_endian)
    };
    let (x, y, z) = (reader("x")?, reader("y")?, reader("z")?);
    let intensity = field("intensity").map(|field| FieldReader::new(field, cloud.point_step, cloud.big_endian)).transpose()?;
    let point_step = cloud.point_step as usize;
    ensure!(point_step > 0, "point_step is 0");
    let row_step = if cloud.row_step == 0 { point_step * cloud.width as usize } else { cloud.row_step as usize };
    ensure!(row_step >= point_step * cloud.width as usize, "row_step {row_step} < width x point_step");
    let mut points = Vec::with_capacity(cloud.width as usize * cloud.height as usize);
    for row in 0..cloud.height as usize {
        for column in 0..cloud.width as usize {
            let start = row * row_step + column * point_step;
            let point = cloud.data.get(start..start + point_step).with_context(|| format!("PointCloud2 data too short at row {row} column {column}"))?;
            let xyz = [x.read(point), y.read(point), z.read(point)];
            if xyz.iter().all(|value| value.is_finite()) {
                points.push(Point { xyz, intensity: intensity.map_or(0.0, |reader| reader.read(point)) });
            }
        }
    }
    Ok(Points { points, has_intensity: intensity.is_some(), header: cloud.header.clone() })
}

/// About `points.len() / keep_every` of the points, in message order: the first in each cell of a voxel grid whose cell
/// size is searched for (on a log scale) until the kept count is within `BUDGET_SLACK` of the budget.
fn thin(points: &[Point], keep_every: u32) -> Cow<'_, [Point]> {
    if keep_every <= 1 || points.len() < 2 {
        return Cow::Borrowed(points);
    }
    let budget = points.len().div_ceil(keep_every as usize);
    let mut low = [f64::INFINITY; 3];
    let mut high = [f64::NEG_INFINITY; 3];
    for point in points {
        for axis in 0..3 {
            low[axis] = low[axis].min(point.xyz[axis]);
            high[axis] = high[axis].max(point.xyz[axis]);
        }
    }
    let extent = (0..3).map(|axis| high[axis] - low[axis]).fold(0.0, f64::max);
    if extent <= 0.0 {
        return points.iter().step_by(keep_every as usize).cloned().collect::<Vec<_>>().into();
    }
    let kept = |cell: f64| -> Vec<usize> {
        let mut seen = std::collections::HashSet::with_capacity(budget * 2);
        (0..points.len())
            .filter(|&index| {
                let xyz = points[index].xyz;
                let key: [i64; 3] = std::array::from_fn(|axis| ((xyz[axis] - low[axis]) / cell).floor() as i64);
                seen.insert(key)
            })
            .collect()
    };
    // a surface-like cloud (a scan, a map's floor and walls) of `budget` cells: extent^2 / cell^2 = budget
    let (mut small, mut large) = (extent * 1e-6, extent);
    let mut cell = extent / (budget as f64).sqrt();
    let mut best: Option<Vec<usize>> = None;
    for _ in 0..BUDGET_SEARCH_STEPS {
        let indices = kept(cell);
        let count = indices.len();
        let closer = best.as_ref().is_none_or(|best| count.abs_diff(budget) < best.len().abs_diff(budget));
        if closer {
            best = Some(indices);
        }
        if (count as f64 - budget as f64).abs() <= BUDGET_SLACK * budget as f64 {
            break;
        }
        // more cells kept than budgeted: grow them
        if count > budget { small = cell } else { large = cell }
        cell = (small * large).sqrt();
    }
    let mut indices = best.unwrap_or_default();
    // still over (a cloud of duplicates): every Nth of what the grid kept
    if indices.len() as f64 > budget as f64 * (1.0 + BUDGET_SLACK) {
        let step = indices.len().div_ceil(budget);
        indices = indices.into_iter().step_by(step).collect();
    }
    indices.into_iter().map(|index| points[index].clone()).collect::<Vec<_>>().into()
}

/// How far from its point budget a thinned cloud may land, and how many grid sizes are tried to get there.
const BUDGET_SLACK: f64 = 0.1;
const BUDGET_SEARCH_STEPS: usize = 12;

/// (origin, scale) so every point fits int16.
fn quantization_grid(low: [f64; 3], high: [f64; 3]) -> ([f64; 3], f64) {
    let half_extent = (0..3).map(|axis| (high[axis] - low[axis]) / 2.0).fold(0.0, f64::max);
    let center: [f64; 3] = std::array::from_fn(|axis| (low[axis] + high[axis]) / 2.0);
    (center, (half_extent / QUANT_MAX).max(f32::MIN_POSITIVE as f64))
}

#[cfg(test)]
pub fn encode(cloud: &PointCloud, quality: f64) -> Result<Vec<u8>> {
    Ok(encode_points(&read_points(cloud)?, quality))
}

/// The allocator's size prior: int16 + zstd roughly quarters a float cloud; thinning sends 1 point in keep_every.
pub fn estimated_bytes(payload_bytes: usize, quality: f64) -> f64 {
    payload_bytes as f64 * 0.25 / keep_every(quality) as f64
}

pub fn encode_points(cloud: &Points, quality: f64) -> Vec<u8> {
    let has_intensity = cloud.has_intensity;
    let source_count = cloud.points.len();
    let keep_every = keep_every(quality);
    let points = thin(&cloud.points, keep_every);
    let mut low = [f64::INFINITY; 3];
    let mut high = [f64::NEG_INFINITY; 3];
    let (mut intensity_low, mut intensity_high) = (f64::INFINITY, f64::NEG_INFINITY);
    for point in points.iter() {
        for axis in 0..3 {
            low[axis] = low[axis].min(point.xyz[axis]);
            high[axis] = high[axis].max(point.xyz[axis]);
        }
        intensity_low = intensity_low.min(point.intensity);
        intensity_high = intensity_high.max(point.intensity);
    }
    let (origin, scale) = if points.is_empty() { ([0.0; 3], 1.0) } else { quantization_grid(low, high) };
    let origin = origin.map(|value| value as f32);
    let scale = scale as f32;
    let intensity_scale = if intensity_high > intensity_low { ((intensity_high - intensity_low) / 255.0) as f32 } else { 1.0 };
    let intensity_min = if points.is_empty() { 0.0 } else { intensity_low as f32 };

    let quantized: Vec<i16> = points
        .iter()
        .flat_map(|point| point.xyz.into_iter().zip(origin).map(|(value, axis_origin)| ((value - axis_origin as f64) / scale as f64).round().clamp(-QUANT_MAX, QUANT_MAX) as i16))
        .collect();
    let fields = cloud.header.add_to(Fields::new())
        .scalar("version", VERSION)
        .scalar("count", points.len() as u32)
        .scalar("sourceCount", source_count as u32)
        .vectors("origin", 3, &origin)
        .scalar("scale", scale)
        .scalar("keepEvery", if points.is_empty() { keep_every } else { (source_count as f64 / points.len() as f64).round().max(1.0) as u32 })
        .scalar("maxError", scale / 2.0)
        .scaled("positions", &origin.map(f64::from), &[scale as f64; 3], &quantized)
        .scalar("intensityMin", intensity_min)
        .scalar("intensityScale", intensity_scale);
    if !has_intensity {
        return fields.build();
    }
    let intensity: Vec<u8> = points.iter().map(|point| ((point.intensity - intensity_min as f64) / intensity_scale as f64).round().clamp(0.0, 255.0) as u8).collect();
    fields.array("intensity", &intensity).build()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::codecs::msgs::{dimos_lcm_pointcloud2, ros2_pointcloud2};
    use crate::utils::tests::fixture;

    type Parse = fn(&[u8]) -> Result<PointCloud<'_>>;

    /// (positions as the client computes them, intensity, scale)
    fn decode(bytes: &[u8]) -> (Vec<[f32; 3]>, Option<Vec<u8>>, f32) {
        let fields = zenoh_gateway::fields::parse(bytes).unwrap();
        assert_eq!(fields["version"].values(), [VERSION as f64]);
        let positions = fields["positions"].values().as_chunks::<3>().0.iter().map(|xyz| xyz.map(|value| value as f32)).collect();
        let intensity = fields.get("intensity").map(|field| field.data.clone());
        (positions, intensity, fields["scale"].values()[0] as f32)
    }

    #[test]
    fn header_rides_along() {
        for (parse, file) in [(ros2_pointcloud2::parse as Parse, "ros2/pointcloud_xyz.cdr"), (dimos_lcm_pointcloud2::parse, "dimos/pointcloud_xyzi.bin")] {
            let payload = fixture(file);
            let cloud = parse(&payload).unwrap();
            let fields = zenoh_gateway::fields::parse(&encode(&cloud, 0.5).unwrap()).unwrap();
            assert_eq!(fields["frameId"].text(), Some(cloud.header.frame_id.as_str()), "{file}");
            assert_eq!(fields["stamp"].values(), [cloud.header.stamp], "{file}");
        }
    }

    #[test]
    fn full_quality_within_bound() {
        for (parse, file, intensity) in [(ros2_pointcloud2::parse as Parse, "ros2/pointcloud_xyz.cdr", false), (dimos_lcm_pointcloud2::parse, "dimos/pointcloud_xyzi.bin", true)] {
            let payload = fixture(file);
            let encoded = encode(&parse(&payload).unwrap(), 1.0).unwrap();
            let (positions, intensities, scale) = decode(&encoded);
            assert_eq!(positions.len(), 20000, "{file}");
            let bound = scale / 2.0 + 1e-5;
            for (index, position) in positions.iter().enumerate() {
                let expected = [(index % 200) as f32 * 0.05, (index / 200) as f32 * 0.05, (index % 7) as f32 * 0.125];
                for axis in 0..3 {
                    assert!((position[axis] - expected[axis]).abs() <= bound, "{file} point {index} axis {axis}: {position:?} vs {expected:?}");
                }
            }
            assert_eq!(intensities.is_some(), intensity);
            if let Some(intensities) = intensities {
                assert!(intensities.iter().enumerate().all(|(index, &value)| value as usize == index % 256));
            }
        }
    }

    #[test]
    fn lower_quality_is_smaller() {
        // a scan-like cloud with noise
        let mut seed = 1u64;
        let mut noise = move || {
            seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            (seed >> 11) as f64 / (1u64 << 53) as f64
        };
        let points = (0..20000).map(|index| Point { xyz: [index as f64 * 0.001 + noise(), noise() * 5.0, noise()], intensity: 0.0 }).collect();
        let cloud = Points { points, has_intensity: false, header: Header::default() };
        let [full, half, lowest] = [1.0, 0.5, 0.0].map(|quality| encode_points(&cloud, quality));
        assert!(half.len() * 4 < full.len() * 3 && lowest.len() < half.len(), "{} / {} / {}", full.len(), half.len(), lowest.len());
        let count = zenoh_gateway::fields::parse(&encode_points(&cloud, 0.5)).unwrap()["count"].values()[0];
        assert!((count - 10000.0).abs() <= BUDGET_SLACK * 10000.0, "half the points, within the budget's slack: {count}");
    }

    #[test]
    fn keep_every_from_quality() {
        let mapped: Vec<u32> = [1.0, 0.9, 0.5, 0.34, 0.25, 0.1, 0.0, -1.0].map(keep_every).to_vec();
        assert_eq!(mapped, [1, 1, 2, 3, 4, 10, MAX_KEEP_EVERY, MAX_KEEP_EVERY]);
    }

    /// Every decoded point is a source point (within the quantization bound), and they come in message order.
    fn assert_kept_unmoved(cloud: &Points, positions: &[[f32; 3]], scale: f32, tolerance: f64, label: &str) {
        let mut next = 0;
        for (sent, position) in positions.iter().enumerate() {
            let found = cloud.points[next..].iter().position(|point| point.xyz.iter().zip(position).all(|(source, decoded)| (*decoded as f64 - source).abs() <= scale as f64 / 2.0 + tolerance));
            let offset = found.unwrap_or_else(|| panic!("{label}: sent point {sent} {position:?} isn't a source point after {next}"));
            next += offset + 1;
        }
    }

    #[test]
    fn thinning_keeps_source_points_unmoved_within_budget() {
        let payload = fixture("ros2/pointcloud_xyz.cdr");
        let cloud = read_points(&ros2_pointcloud2::parse(&payload).unwrap()).unwrap();
        for (quality, keep) in [(0.5, 2usize), (1.0 / 3.0, 3), (0.0, MAX_KEEP_EVERY as usize)] {
            let encoded = encode_points(&cloud, quality);
            let (positions, _, scale) = decode(&encoded);
            let budget = 20000usize.div_ceil(keep) as f64;
            assert!((positions.len() as f64 - budget).abs() <= BUDGET_SLACK * budget, "q {quality}: {} points for a budget of {budget}", positions.len());
            let reported = zenoh_gateway::fields::parse(&encoded).unwrap()["keepEvery"].values()[0];
            assert!((reported - keep as f64).abs() <= 1.0, "q {quality}: keepEvery {reported}");
            assert_kept_unmoved(&cloud, &positions, scale, 1e-5, &format!("q {quality}"));
        }
    }

    #[test]
    fn thinning_spreads_points_over_space() {
        // a ring-ordered scan: 16 rings of 1250 points, ring r at radius 1 + r; 1-in-16 in message order keeps every
        // 16th point of each ring (the outer rings get as few points per metre as the inner ones get per 16 cm)
        let points: Vec<Point> = (0..16)
            .flat_map(|ring| (0..1250).map(move |step| {
                let angle = step as f64 / 1250.0 * std::f64::consts::TAU;
                let radius = 1.0 + ring as f64;
                Point { xyz: [radius * angle.cos(), radius * angle.sin(), 0.0], intensity: 0.0 }
            }))
            .collect();
        let cloud = Points { points, has_intensity: false, header: Header::default() };
        let (positions, _, _) = decode(&encode_points(&cloud, 0.25));
        // nearest kept neighbour of each kept point: the grid evens out the spacing across rings
        let spacing = |positions: &[[f32; 3]]| -> (f64, f64) {
            let nearest: Vec<f64> = positions.iter().enumerate().map(|(index, a)| positions.iter().enumerate().filter(|(other, _)| *other != index).map(|(_, b)| ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2)).sqrt() as f64).fold(f64::INFINITY, f64::min)).collect();
            (nearest.iter().copied().fold(f64::INFINITY, f64::min), nearest.iter().copied().fold(0.0, f64::max))
        };
        let (closest, farthest) = spacing(&positions);
        let stride: Vec<[f32; 3]> = cloud.points.iter().step_by(4).map(|point| point.xyz.map(|value| value as f32)).collect();
        let (stride_closest, stride_farthest) = spacing(&stride);
        assert!(farthest / closest < stride_farthest / stride_closest, "grid {closest:.3}..{farthest:.3} vs stride {stride_closest:.3}..{stride_farthest:.3}");
    }

    #[test]
    fn units_do_not_matter() {
        // the same cloud in mm, m and km: the same number kept, the same error relative to the cloud's size
        let points: Vec<Point> = (0..5000).map(|index| Point { xyz: [(index % 100) as f64, (index / 100) as f64, (index % 7) as f64 * 0.3], intensity: 0.0 }).collect();
        let mut counts = Vec::new();
        for unit in [1e-3, 1.0, 1e3] {
            let cloud = Points { points: points.iter().map(|point| Point { xyz: point.xyz.map(|value| value * unit), intensity: 0.0 }).collect(), has_intensity: false, header: Header::default() };
            for quality in [1.0, 0.5, 0.0] {
                let (positions, _, scale) = decode(&encode_points(&cloud, quality));
                counts.push((quality.to_bits(), positions.len()));
                assert_kept_unmoved(&cloud, &positions, scale, 99.0 * unit * 1e-6, &format!("unit {unit} q {quality}"));
            }
        }
        for (quality, count) in &counts {
            assert!(counts.iter().filter(|(other, _)| other == quality).all(|(_, other)| other == count), "{counts:?}");
        }
    }

    #[test]
    fn skips_nan_and_honors_layout() {
        // big endian, padded point_step 20, row_step with 4 bytes of padding, one NaN point
        let fields = vec![
            PointField { name: "z".into(), offset: 8, datatype: 7, count: 1 },
            PointField { name: "x".into(), offset: 0, datatype: 8, count: 1 },
            PointField { name: "y".into(), offset: 12, datatype: 3, count: 1 },
        ];
        let mut data = Vec::new();
        for (x, y, z) in [(1.0f64, 2i16, 3.0f32), (f64::NAN, 0, 0.0), (-1.0, -2, -3.0)] {
            data.extend_from_slice(&x.to_be_bytes());
            data.extend_from_slice(&z.to_be_bytes());
            data.extend_from_slice(&y.to_be_bytes());
            data.extend_from_slice(&[0; 6]);
        }
        data.extend_from_slice(&[0; 4]);
        let cloud = PointCloud { header: Header::new(0, 0, "lidar".into(), None), height: 1, width: 3, fields, big_endian: true, point_step: 20, row_step: 64, data: &data };
        let (positions, intensity, scale) = decode(&encode(&cloud, 1.0).unwrap());
        assert!(intensity.is_none());
        assert_eq!(positions.len(), 2);
        for (position, expected) in positions.iter().zip([[1.0, 2.0, 3.0], [-1.0, -2.0, -3.0]]) {
            for axis in 0..3 {
                assert!((position[axis] - expected[axis]).abs() <= scale, "{position:?}");
            }
        }
    }
}
