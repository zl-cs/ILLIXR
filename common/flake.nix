{
  outputs = { self, nixpkgs }:
  let
    pkgs = nixpkgs.legacyPackages.x86_64-linux;
  in {
    packages.x86_64-linux.illixr-runtime = pkgs.clangStdenv.mkDerivation {
      pname = "illixr-common";
      version = "2.2.0-latest";
      src = ./.;
      phases = "buildPhase";

      buildPhase = ''
        mkdir -p $out/include/common
	cd $src
	fd '.*\.hpp' . --exec mkdir -p $out/include/common/'{//}'
	fd '.*\.hpp' . --exec cp '{}' $out/include/common/'{//}'
      '';

      buildInputs = [
        pkgs.fd
      ];

      propagatedBuildInputs = [
        pkgs.opencv3
      ];
    };

    defaultPackage.x86_64-linux = self.packages.x86_64-linux.illixr-runtime;
  };
}
