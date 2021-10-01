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

# Build audio_pipeline
make -C benchmark/audio_pipeline solo.opt

mkdir -p data

# Build ElasticFusion
./benchmark/ElasticFusion/install_deps.sh
./benchmark/ElasticFusion/build.sh
wget -O data/dyson_lab.klg www.doc.ic.ac.uk/~sleutene/datasets/elasticfusion/dyson_lab.klg

# Build Hologram
make -C benchmark/HOTlab/C/source all

# Build visual_postprocessing
make -C benchmark/visual_postprocessing/src all
wget -O data/test.png https://upload.wikimedia.org/wikipedia/commons/thumb/3/3f/Internet_map_1024_-_transparent%2C_inverted.png/1024px-Internet_map_1024_-_transparent%2C_inverted.png

# Install dependencies for RITnet
. $HOME/miniconda3/etc/profile.d/conda.sh
conda env create --name RITnet --file benchmark/RITnet/environment.yml

# Build OpenVINS
./benchmark/open_vins_ws/src/open_vins/install_deps.sh
