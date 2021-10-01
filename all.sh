#!/usr/bin/env sh

set -e -x
cd "$(dirname ""${0}"")"

# Run audio encode/decode
cd benchmark/audio_pipeline
./solo.opt encode 1
./solo.opt decode 1
cd ..

# Run ElasticFusion
./benchmark/ElasticFusion/ElasticFusion -l data/dyson_lab.klg

# Run Hologram
./benchmark/HOTlab/C/source/hologram

# Build and run visual_postprocessing
./benchmark/visual_postprocessing/bin/fbo data/test.png

# Run RITnet
$HOME/miniconda3/bin/conda run --name RITnet --cwd benchmark/RITnet python3 test.py
