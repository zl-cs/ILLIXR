{ pkgs ? import <nixpkgs> {} }:

  pkgs.mkShell {
    shellHook = ''
			  	export LD_LIBRARY_PATH=$(nixGLNvidia printenv LD_LIBRARY_PATH):$LD_LIBRARY_PATH
			  '';
    buildInputs = [
				pkgs.pkgconfig
				pkgs.clang_10
				pkgs.opencv3
				pkgs.gnumake
				pkgs.cmake
				pkgs.eigen
				pkgs.boost167
				pkgs.boost167.out
				pkgs.boost167.dev
				pkgs.blas
				pkgs.glew
				pkgs.glfw
				pkgs.libGL
				pkgs.freeglut
				pkgs.cudaPackages.cudatoolkit_10_1
	];
}
