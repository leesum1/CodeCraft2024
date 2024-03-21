with import <nixpkgs> {}; {
  qpidEnv = stdenvNoCC.mkDerivation {
    name = "my-gcc8-environment";
    buildInputs = [
        gcc7
    ];
  };
}