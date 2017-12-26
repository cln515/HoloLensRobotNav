/*
Copyright (c) 2017 Ryoichi Ishikawa. All rights reserved.

This software is released under the MIT License.
http://opensource.org/licenses/mit-license.php
*/

#pragma once


#include <vector>

#ifndef FLOOR_DETECTOR
#define FLOOR_DETECTOR

void FloorDetection(Platform::Array<unsigned char>^ buffer, int rowpitch, int height, double scale, double& HoloHeight, Eigen::Vector3d& floorpt);

#endif
