#!/bin/bash

set -e -x
cd "$(dirname ${0})"

if [ ! -d apitrace_src/ ]; then
	git clone https://github.com/apitrace/apitrace apitrace_src
	cmake -S apitrace_src -B apitrace_src/build -D CMAKE_BUILD_TYPE=RelWithDebInfo
	make -C apitrace_src/build
fi

mkdir -p output

if [ -n "${1}" ]; then
	configs=("${1}")
else
	# TODO: in the future, scan over all approx knobs
	configs=(
		ground_truth_ssim
		approx_config_ssim
		pose_accuracy_ssim
		approx_config_pose_accuracy
	)
fi

for config in "${configs[@]}"; do
	./runner.sh "configs/${config}.yaml" > "output/${config}.log"
	if [ -f trace ]; then
		mv trace "output/${config}.trace"
	fi
	if [ -f pose_accuracy.csv ]; then
		mv pose_accuracy.csv "output/${config}.pose_accuracy.csv"
	fi
done
