#pragma once


#include <vector>

#ifndef FLOOR_DETECTOR
#define FLOOR_DETECTOR

void FloorDetection(Platform::Array<unsigned char>^ buffer, int rowpitch, int height, double scale, double& HoloHeight, Eigen::Vector3d& floorpt);

#endif
