#!/usr/bin/env sh

set -e -x
cd "$(dirname ""${0}"")"

# Build audio_pipeline
make -C benchmarks/audio_pipeline solo.opt
cd benchmarkd/audio_pipeline

# Run encode
./solo.opt encode 1024

# Run decode
./solo.opt decode 1024

cd ..

cd benchmarks/ElasticFusion
./build.sh
