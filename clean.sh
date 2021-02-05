#!/bin/bash

plugins=(
	offline_imu_cam/
	../open_vins/
	gtsam_integrator/
	pose_prediction/
	gldemo/
	debugview/
	timewarp_gl/
	runtime/
)
for plugin in "${plugins[@]}"; do
	make -C "${plugin}" clean
done
