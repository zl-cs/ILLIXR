#!/usr/bin/env sh

set -e -x
cd "$(dirname ""${0}"")"

# Install Intel VTune (profiling tool), note sure where else to put this
wget -qO - https://apt.repos.intel.com/intel-gpg-keys/GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB | sudo apt-key add -
sudo add-apt-repository "deb https://apt.repos.intel.com/oneapi all main"
sudo apt-get update
sudo apt-get install -y intel-basekit

# This allows us to use perf without escalating priveleges
sudo sh -c 'echo 0 > /proc/sys/kernel/perf_event_paranoid'

# If we don't specify which clang, assume clang-10
if ! which clang++; then
    sudo ln -s $(which clang++-10) /usr/bin/local/clang++
fi

./benchmarks/ElasticFusion/install_deps.sh
