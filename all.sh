#!/usr/bin/env sh

set -e -x
cd "$(dirname ""${0}"")"

# Run audio encode/decode
# env -C benchmark/audio_pipeline AUDIO_ROOT=benchmark/audio_pipeline LD_LIBRARY_PATH=$PWD/benchmark/audio_pipeline/portaudio/lib/.libs:$PWD/benchmark/audio_pipeline: ./solo.opt.exe 2000 encode
env -C benchmark/audio_pipeline ./solo.opt 2000 encode
env -C benchmark/audio_pipeline ./solo.opt 2000 decode

# Run ElasticFusion
./benchmark/ElasticFusion/ElasticFusion -l data/dyson_lab.klg

# Run Hologram
./benchmark/HOTlab/C/source/hologram 1

# Build and run visual_postprocessing
./benchmark/visual_postprocessing/bin/fbo ./benchmark/visual_postprocessing/src/landscape.png

# Run OpenVINS
./benchmark/open_vins/build/RelWithDebInfo/ov_msckf/run_illixr_msckf ../ILLIXR/data1/cam0/data.csv data/corrected_cam1_data.csv ../ILLIXR/data1/imu0/data.csv ../ILLIXR/data1/cam0/data ../ILLIXR/data1/cam1/data

# Run RITnet
$HOME/miniconda3/bin/conda run --name RITnet --cwd benchmark/RITnet python3 test.py
