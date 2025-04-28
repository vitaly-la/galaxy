{
  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs";
    flake-utils.url = "github:numtide/flake-utils";
  };

  outputs = { self, flake-utils, nixpkgs }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = import nixpkgs { inherit system; };
      in
      {
        packages.default = pkgs.rustPlatform.buildRustPackage rec {
          name = "galaxy";

          cargoLock.lockFile = ./Cargo.lock;

          src = ./.;

          nativeBuildInputs = with pkgs; [ makeWrapper rustfmt clippy rust-analyzer ];

          buildInputs = with pkgs; [
            xorg.libX11
            xorg.libXcursor
            xorg.libXrandr
            xorg.libXi
            libGL
            llvmPackages.libclang.lib
          ];

          LD_LIBRARY_PATH = "${pkgs.lib.makeLibraryPath buildInputs}";

          postInstall = ''
            wrapProgram $out/bin/galaxy --set LD_LIBRARY_PATH "${LD_LIBRARY_PATH}"
          '';
        };
      }
    );
}
