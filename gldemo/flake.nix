{
  outputs = { self, nixpkgs }: {
    packages.x86_64-linux.illixr-gldemo =
    with import nixpkgs { system = "x86_64-linux"; };
    clangStdenv.mkDerivation {
      pname = "illixr-gldemo";
      version = "2.2.1-latest";
      src = self;
      configurePhase = ''
        export NIX_FLAKES=ON
      '';
      buildPhase = ''
        make -C $src/gldemo plugin.dbg.so
      '';
      installPhase = ''
        # So far the installation is handled by 'Makefile's,
        # but please keep this 'installPhase' and comments,
        # or please disable 'installPhase'.
      '';
      buildInputs = [
        glew
        boost
        opencv3
        eigen
        glfw
        x11
      ];
    };
    defaultPackage.x86_64-linux = self.packages.x86_64-linux.illixr-gldemo;
  };
}
