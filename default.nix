{ pkgs ? import <nixpkgs> { } }:
pkgs.rustPlatform.buildRustPackage rec {
  pname = "galaxy";
  version = "0.1.0";

  nativeBuildInputs = with pkgs; [ rustfmt clippy rust-analyzer makeWrapper ];

  buildInputs = with pkgs; [
    xorg.libX11
    xorg.libXcursor
    xorg.libXrandr
    xorg.libXi
    libGL
  ];

  cargoLock.lockFile = ./Cargo.lock;
  src = pkgs.lib.cleanSource ./.;

  LD_LIBRARY_PATH = "${pkgs.lib.makeLibraryPath buildInputs}";

  postInstall = ''
    wrapProgram $out/bin/galaxy --set LD_LIBRARY_PATH "${pkgs.lib.makeLibraryPath buildInputs}"
  '';
}
