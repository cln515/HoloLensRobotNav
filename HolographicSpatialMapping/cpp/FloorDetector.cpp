#include "FloorDetector.h"
#include <Eigen\Eigen>
#include <Eigen\Core>
#include <Eigen\Dense>
#include "pch.h"
void FloorDetection(Platform::Array<unsigned char>^ buffer,int rowpitch,int height,double scale,double& HoloHeight, Eigen::Vector3d& floorpt) {
	//Buffer 2 Array

	float * imageData = (float*)buffer->Data;
	
	
	//get plane and Height
	//around 1m x 1m
	int dwidth = 1.0 / scale;
	//ransac
	int ransac_max = 100;
	std::vector<Eigen::Vector3d> points;
	std::vector<unsigned int> idces;
	Eigen::Vector3d bestn, bestp;
	bestn << 1, 1, 1;
	unsigned int cnt = 0;
	int maxcnt = -1;
	for (int x = rowpitch / sizeof(float) / 2;x<rowpitch / 2 / sizeof(float) + dwidth;x++) {
		for (int y = height / 2;y<height / 2 + dwidth;y++) {
			Eigen::Vector3d p_temp;
			p_temp << x*scale, y*scale, imageData[x + y*(rowpitch / sizeof(float))] * 3.0;
			points.push_back(p_temp);
			idces.push_back(cnt);
			cnt++;
		}
	}
	std::vector<unsigned int> bestlist;
	for (int ransac_t = 0;ransac_t<ransac_max;ransac_t++) {
		random_shuffle(idces.begin(), idces.end());
		std::vector<unsigned int> candlist;
		Eigen::Vector3d v01, v02, nfloor, cand_p;
		cand_p = points.at(idces.at(0));
		v01 = points.at(idces.at(1)) - cand_p;
		v02 = points.at(idces.at(2)) - cand_p;
		nfloor = v01.cross(v02);
		nfloor = nfloor.normalized();
		int inlcnt = 0;
		for (int idx = 3;idx<idces.size();idx++) {
			Eigen::Vector3d targp = points.at(idces.at(idx)) - cand_p;

			double err = abs(targp.dot(nfloor));
			if (err<0.005) {
				//inlier
				candlist.push_back(idces.at(idx));
				inlcnt++;
			}
		} if (maxcnt<inlcnt) {
			maxcnt = inlcnt;
			bestn = nfloor;
			bestp = cand_p;
			bestlist = std::vector<unsigned int>(candlist);
		}
	}
	//plane fitting
	//solve least square problem
	//ax+by+z+d=0: ax+by+d=-z
	Eigen::MatrixXd A(bestlist.size(), 3);
	Eigen::VectorXd B(bestlist.size());
	for (int idx = 0;idx<bestlist.size();idx++) {
		Eigen::Vector3d targp = points.at(idces.at(idx));
		A(idx, 0) = targp(0);//x
		A(idx, 1) = targp(1);//y
		A(idx, 2) = 1;//1
		B(idx) = -targp(2);//-z
	}
	Eigen::Vector3d ansX = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(B);

	//rendering: 1.0m upper from hololens - 2.0m lower from hololens (3.0m range)
	//hololens point (scale*(dwidth/2),scale*(dwidth/2),1.0)
	//ax+by+z+d=0: n<<a,b,1
	Eigen::Vector3d phl;
	bestn << ansX(0), ansX(1), 1;
	bestp << scale*(rowpitch / sizeof(float) / 2), scale*(rowpitch / sizeof(float) / 2), -(ansX(0) + ansX(1))*scale*(rowpitch / sizeof(float) / 2) - ansX(2);
	bestn = bestn.normalized();
	if (bestn(2)<0)bestn = -bestn;

	//rendering: 1.0m upper from hololens - 2.0m lower from hololens (3.0m range)
	//hololens point (scale*(dwidth/2),scale*(dwidth/2),1.0)
	phl << scale*(rowpitch / sizeof(float) / 2), scale*(rowpitch / sizeof(float) / 2), 1.0;
	HoloHeight = -(phl - bestp).dot(bestn);
	floorpt = -HoloHeight*bestn;//hololens 2 floor

};