{
    description = "zenoh-dimos-codecs: zenoh-gateway codecs for ROS 2 and dimos messages (crate2nix builds, native and aarch64 Linux)";

    # zenoh-gateway's lib.crossRust: its nixpkgs / rust-overlay pins, so crates are shared with the other zenoh-gateway flakes
    inputs.zenoh-gateway.url = "github:dimensionalOS/zenoh-gateway";

    outputs = { self, zenoh-gateway }: {
        # the library through nix_smoke_test/: a loopback server with every codec
        packages = zenoh-gateway.lib.eachSystem (system:
            let built = zenoh-gateway.lib.crossRustPackages { name = "zenoh-dimos-codecs-example"; inherit system; cargoNix = ./nix_smoke_test/Cargo.nix; };
            in built // { default = built.zenoh-dimos-codecs-example; });
        devShells = zenoh-gateway.devShells;
    };
}
