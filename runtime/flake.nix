{
  outputs = { self, nixpkgs }:
  let
    pkgs = nixpkgs.legacyPackages.x86_64-linux;
  in {
    packages.x86_64-linux.illixr-runtime = pkgs.stdenv.mkDerivation {
      pname = "illixr-runtime";
      version = "2.2.0-latest";
      src = ./.;
      phases = "buildPhase";

      buildPhase = ''
        mkdir -p $out/bin
        # TODO: build goes here
      '';

      # only available at compile-time
      buildInputs = [
	# TODO: clang compiler
	# TODO: common headers (will have to make a flake for ../common)
	# TODO: ... other stuff
      ];

      # available at compile- and run-time
      propagatedBuildInputs = [
        # TODO: dynamic libraries
	# TODO: ... other stuff

	# plugins, dataset, monado, and OpenXR app, will be pulled in by a parent package;
	# No need for them here.
      ];
    };

    defaultPackage.x86_64-linux = self.packages.x86_64-linux.illixr-runtime;
  };
}
