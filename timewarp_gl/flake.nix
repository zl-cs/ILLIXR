{
  outputs = { self, nixpkgs }: {
    packages.x86_64-linux.illixr-timewarp_gl =
    with import nixpkgs { system = "x86_64-linux"; };
    clangStdenv.mkDerivation {
      pname = "illixr-timewarp_gl";
      version = "2.2.1-latest";
      src = self;
      configurePhase = ''
        export NIX_FLAKES=ON
      '';
      buildPhase = ''
        mkdir -p $out/lib
        mkdir -p $out/bin
        mkdir -p $out/obj
        make -C $src/timewarp_gl main.dbg.exe
      '';
      installPhase = ''
        # So far the installation is handled by 'Makefile's,
        # but please keep this 'installPhase' and comments,
        # or please disable 'installPhase'.
      '';
      buildInputs = [
      ];
    };
    defaultPackage.x86_64-linux = self.packages.x86_64-linux.illixr-timewarp_gl;
  };
}
