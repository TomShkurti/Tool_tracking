/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016 Case Western Reserve University
 *	
 *	Orhan Ozguner <oxo31@case.edu>
 *	Russell Jackson <rcj33@case.edu>
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
 *   * Neither the name of Case Western Reserve University, nor the names of its
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
 */

#ifndef PARTICLEFILTER_H
#define PARTICLEFILTER_H

#include <vector>
#include <stdio.h>
#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <stdlib.h>
#include <math.h>

#include <Eigen/Eigen>

#include <string>
#include <cstring>

#include <tool_model_lib/tool_model.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/image_encodings.h>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>

#include <vesselness_image_filter_cpu/vesselness_lib.h>
#include <boost/random/normal_distribution.hpp>

#include <geometry_msgs/Transform.h>

class KalmanFilter {

private:

	ros::NodeHandle nh;
	
	Eigen::Matrix<double, 14, 14> A;
	Eigen::Matrix<double, 14, 14> B;
	Eigen::Matrix<double, 14, 14> C;
	
	Eigen::Matrix<double, 14, 14> R;
	Eigen::Matrix<double, 14, 14> Q;
	
	Eigen::Matrix<double, 14, 1> mu;
	Eigen::Matrix<double, 14, 14> sigma;

public:

	/*
	* The default constructor
	*/
	KalmanFilter(ros::NodeHandle *nodehandle);

	/*
	 * The deconstructor 
	 */
	~KalmanFilter();

	/***consider getting a timer to debug***/
	// void timerCallback(const ros::TimerEvent&);

	/*
	 * This is the main function for tracking the needle. This function is called and it syncs all of the functions
	*/
	std::vector<cv::Mat> trackingTool(
		const cv::Mat &bodyVel,
		const cv::Mat &segmented_left,
		const cv::Mat &segmented_right,
		const cv::Mat &P_left,
		const cv::Mat &P_right,
		const Eigen::Matrix<double, 14, 1> & ut,
		const Eigen::Matrix<double, 14, 1> & zt
	);
};

#endif
