#!/usr/bin/env sh

set -e -x
cd "$(dirname ""${0}"")"

# Install Intel VTune (profiling tool), note sure where else to put this
wget -qO - https://apt.repos.intel.com/intel-gpg-keys/GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB | sudo apt-key add -
sudo add-apt-repository "deb https://apt.repos.intel.com/oneapi all main"
sudo apt-get update
sudo apt-get install -y intel-basekit

# Install perf
sudo apt-get install -y linux-tools-common
sudo sh -c 'echo 0 > /proc/sys/kernel/perf_event_paranoid'
sudo sh -c 'echo 0 > /proc/sys/kernel/kptr_restrict'

# If we don't specify which clang, assume clang-10
if ! which clang++; then
    sudo ln -s $(which clang++-10) /usr/bin/local/clang++
fi

mkdir -p data

# Build audio_pipeline
# ln -s common benchmark/audio_pipeline
# make -C benchmark/audio_pipeline solo.opt.exe
make -C benchmark/audio_pipeline solo.opt

# Build ElasticFusion
./benchmark/ElasticFusion/install_deps.sh
./benchmark/ElasticFusion/build.sh
wget -O data/dyson_lab.klg www.doc.ic.ac.uk/~sleutene/datasets/elasticfusion/dyson_lab.klg

# Build Hologram
make -C benchmark/HOTlab/C/source all

# Build visual_postprocessing
make -C benchmark/visual_postprocessing/src all

# Install dependencies for RITnet
. $HOME/miniconda3/etc/profile.d/conda.sh
conda env create --name RITnet --file benchmark/RITnet/environment.yml

# Build OpenVINS
head -n 1711 ../ILLIXR/data1/cam1/data.csv > data/corrected_cam1_data.csv
make -C benchmark/open_vins opt
