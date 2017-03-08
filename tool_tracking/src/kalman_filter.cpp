/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016 Case Western Reserve University
 *
 *	Ran Hao <rxh349@case.edu>
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *	 notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *	 copyright notice, this list of conditions and the following
 *	 disclaimer in the documentation and/or other materials provided
 *	 with the distribution.
 *   * Neither the name of Case Western Reserve Univeristy, nor the names of its
 *	 contributors may be used to endorse or promote products derived
 *	 from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <tool_tracking/kalman_filter.h>  //everything inside here
#include <opencv2/calib3d/calib3d.hpp>

using namespace std;

KalmanFilter::KalmanFilter(ros::NodeHandle *nodehandle) : nh(*nodehandle){
	ROS_INFO("Hello! Being initialized!");
	A = Eigen::Matrix<double, 14, 14>::Identity(14, 14);//The robot is immobile when not commanded.
	B = Eigen::Matrix<double, 14, 14>::Identity(14, 14) * 5.0;//Disregarding noise, the robot will move by an amount proportional to its velocity. Velocity is computed by taking the difference of values outputted five times per loop iteration, so over one loop iteration the robot can be expected to move by five times the velocity.
	C = Eigen::Matrix<double, 14, 14>::Identity(14, 14);//The potentiometers accurately reflect joint movement in the same scale, disregarding noise.
	
	R = Eigen::Matrix<double, 14, 14>::Identity(14, 14) * 0.037;//Rough averages of error values from the paper.
	Q = Eigen::Matrix<double, 14, 14>::Identity(14, 14) * 0.00038;
	
	mu = Eigen::Matrix<double, 14, 1>::Zero(14, 1);
	sigma = Eigen::Matrix<double, 14, 14>::Identity(14, 14) * 0.037;
};

KalmanFilter::~KalmanFilter() {

};

std::vector<cv::Mat> KalmanFilter::trackingTool(
	const cv::Mat &bodyVel,
	const cv::Mat &segmented_left,
	const cv::Mat &segmented_right,
	const cv::Mat &P_left,
	const cv::Mat &P_right,
	const Eigen::Matrix<double, 14, 1> & ut,
	const Eigen::Matrix<double, 14, 1> & zt
){

	Eigen::Matrix<double, 14, 1> mubar = A * mu + B * ut;
	Eigen::Matrix<double, 14, 14> sigmabar = (A * sigma) * A.transpose() + R;
	
	Eigen::Matrix<double, 14, 14> K = sigmabar * C.transpose() * (C * sigmabar * C.transpose() + Q).inverse();
	mu = mubar + K * (zt - C * mubar);
	sigma = (Eigen::Matrix<double, 14, 14>::Identity(14, 14) - K * C) * sigmabar;

	std::vector<cv::Mat> trackingImages;
	trackingImages.resize(2);
	trackingImages[0] = P_left;
	trackingImages[1] = P_right;

	return trackingImages;
};
