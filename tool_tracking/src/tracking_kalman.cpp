#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <cwru_opencv_common/projective_geometry.h>
#include <tool_tracking/kalman_filter.h>
#include <cwru_davinci_interface/davinci_interface.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/Float64MultiArray.h"

bool freshImage;
bool freshCameraInfo;
bool freshVelocity;
bool freshJoints;

double Arr[6];

using namespace std;
using namespace cv_projective;

std::vector<cv::Mat> trackingImgs;  ///this should be CV_32F
std::vector<std::vector<double> > joints;
std::vector<std::vector<double> > velocities;

std::vector<std::vector<double> > cmds;
std::vector<std::vector<double> > cmds_prev;

void newCommandCallback1(const sensor_msgs::JointState::ConstPtr& incoming){
	std::vector<double> positions = incoming->position;
	for(int i = 0; i < 7; i++){
		cmds[0][i] = positions[i];
	}
}

void newCommandCallback2(const sensor_msgs::JointState::ConstPtr& incoming){
	std::vector<double> positions = incoming->position;
	for(int i = 0; i < 7; i++){
		cmds[1][i] = positions[i];
	}
}

void newImageCallback(const sensor_msgs::ImageConstPtr &msg, cv::Mat *outputImage) {
	//ROS_ERROR("ImgCallback");
	//Save this for later.
	//Sensor update goes here.
	//Actually do the sensor update here?
	//cv_bridge::CvImagePtr cv_ptr;
	//try {
	//	//cv::Mat src =  cv_bridge::toCvShare(msg,"32FC1")->image;
	//	//outputImage[0] = src.clone();
	//	cv_ptr = cv_bridge::toCvCopy(msg);
	//	outputImage[0] = cv_ptr->image;
	//	freshImage = true;
	//}
	//catch (cv_bridge::Exception &e) {
	//	ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
	//}
}

// receive body velocity
void arrayCallback(const std_msgs::Float64MultiArray::ConstPtr &array) {
	ROS_ERROR("array callback");
	//What does this do?
	//Is this a control input?
	//I don't think it does anything.
	int i = 0;
	// print all the remaining numbers
	for (std::vector<double>::const_iterator it = array->data.begin(); it != array->data.end(); ++it) {
		Arr[i] = *it;
		i++;
	}

	freshVelocity = true;

}

void timerCB(const ros::TimerEvent &) {

//	std::vector<cv::Mat> disp;
//	disp.resize(2);
//
//	for (int j(0); j<2; j++)
//	{
//		convertSegmentImageCPUBW(trackingImgs[j],disp[j]);  //what is this?
//	}

//	if(freshImage && freshCameraInfo && freshVelocity){
//		cv::imshow( "Trancking Image: LEFT", disp[0]);
//		cv::imshow( "Trancking Image: RIGHT", disp[1]);
//	}

	// cv::waitKey(10);

}

cv::Mat segmentation(cv::Mat &InputImg) {
	//This is just an IP utility function?

	//ROS_INFO("SEGMENTATION!!!!");
	cv::Mat src, src_gray;
	cv::Mat grad;

	cv::Mat res;
	src = InputImg;

	resize(src, src, cv::Size(), 1, 1);

	double lowThresh = 43;

	cv::cvtColor(src, src_gray, CV_BGR2GRAY);

	blur(src_gray, src_gray, cv::Size(3, 3));

	Canny(src_gray, grad, lowThresh, 4 * lowThresh, 3); //use Canny segmentation

	grad.convertTo(res, CV_32FC1);

	return res;
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "tracking_node");

	ros::NodeHandle nh;
	/******  initialization  ******/
	KalmanFilter UKF(&nh);//Replace me with the UKF

	freshCameraInfo = false;
	freshImage = false;
	freshVelocity = false;
	freshJoints = false;

	cv::Mat seg_left;
	cv::Mat seg_right;

	cv::Mat bodyVel = cv::Mat::zeros(6, 1, CV_64FC1);

	trackingImgs.resize(2);
	
	//Resturcture this for Kalman functionality... actually, probably better to put all this in the UKF header itself.
	/****TODO: Temp Projection matrices****/
	cv::Mat P_l(3, 4, CV_64FC1);
	P_l.at<double>(0, 0) = 893.7852590197848;
	P_l.at<double>(1, 0) = 0;
	P_l.at<double>(2, 0) = 0;

	P_l.at<double>(0, 1) = 0;
	P_l.at<double>(1, 1) = 893.7852590197848;
	P_l.at<double>(2, 1) = 0;

	P_l.at<double>(0, 2) = 288.4443244934082; // horiz
	P_l.at<double>(1, 2) = 259.7727756500244; //verticle
	P_l.at<double>(2, 2) = 1;

	P_l.at<double>(0, 3) = 0;
	P_l.at<double>(1, 3) = 0;
	P_l.at<double>(2, 3) = 0;

	cv::Mat P_r(3, 4, CV_64FC1);
	P_r.at<double>(0, 0) = 893.7852590197848;
	P_r.at<double>(1, 0) = 0;
	P_r.at<double>(2, 0) = 0;

	P_r.at<double>(0, 1) = 0;
	P_r.at<double>(1, 1) = 893.7852590197848;
	P_r.at<double>(2, 1) = 0;

	P_r.at<double>(0, 2) = 288.4443244934082; // horiz
	P_r.at<double>(1, 2) = 259.7727756500244; //verticle
	P_r.at<double>(2, 2) = 1;

	P_r.at<double>(0, 3) = 4.732953897952732;
	P_r.at<double>(1, 3) = 0;
	P_r.at<double>(2, 3) = 0;

	clock_t t;
	double avg_tim = 0.0;
	int count = 1;
	
	cmds.resize(2);
	cmds[0].resize(7);
	cmds[1].resize(7);
	cmds_prev.resize(2);
	cmds_prev[0].resize(7);
	cmds_prev[1].resize(7);
	velocities.resize(2);
	velocities[0].resize(7);
	velocities[1].resize(7);
	for(int i = 0; i < 7; i++){
		cmds[0][i] = 0.0;
		cmds[1][i] = 0.0;
		cmds_prev[0][i] = 0.0;
		cmds_prev[1][i] = 0.0;
	}

	/*** Timer set up ***/
	ros::Rate loop_rate(50);
	ros::Timer timer = nh.createTimer(ros::Duration(0.01), timerCB); //show images

	/*** Subscribers, velocity, stream images ***/

	ros::Subscriber sub3 = nh.subscribe("/bodyVelocity", 100, arrayCallback);

	const std::string leftCameraTopic("/davinci_endo/left/camera_info");
	const std::string rightCameraTopic("/davinci_endo/left/camera_info");
	cameraProjectionMatrices cameraInfoObj(nh, leftCameraTopic, rightCameraTopic);
	ROS_INFO("---- Connected to camera info -----");

	//TODO: get image size from camera model, or initialize segmented images,

	cv::Mat rawImage_left = cv::Mat::zeros(480, 640, CV_32FC1);
	cv::Mat rawImage_right = cv::Mat::zeros(480, 640, CV_32FC1);

	image_transport::ImageTransport it(nh);
	image_transport::Subscriber img_sub_l = it.subscribe(
		"/davinci/left_camera/image_raw",
		1,
		boost::function<void(const sensor_msgs::ImageConstPtr &)>(boost::bind(newImageCallback, _1, &rawImage_left))
	);
	image_transport::Subscriber img_sub_r = it.subscribe(
		"/davinci/right_camera/image_raw",
		1,
		boost::function<void(const sensor_msgs::ImageConstPtr &)>(boost::bind(newImageCallback, _1, &rawImage_right))
	);
	
	//Get the joint positions
	davinci_interface::init_joint_feedback(nh);
	freshJoints = davinci_interface::get_fresh_robot_pos(joints);
	
	//And get the joint commands, which will subsequently become velocities.
	ros::Subscriber com_s1 = nh.subscribe("/dvrk/PSM1/set_position_joint", 10, newCommandCallback1);
	ros::Subscriber com_s2 = nh.subscribe("/dvrk/PSM2/set_position_joint", 10, newCommandCallback2);

	ROS_INFO("---- done subscribe -----");

	/***testing segmentation images***/
	cv::Size size(640, 480);
	seg_left = cv::imread("/home/tom/catkin_ws/src/project_stuff/Tool_tracking/tool_tracking/left.png",CV_LOAD_IMAGE_GRAYSCALE );
	seg_right = cv::imread("/home/tom/catkin_ws/src/project_stuff/Tool_tracking/tool_tracking/right.png",CV_LOAD_IMAGE_GRAYSCALE );

	cv::resize(seg_left, seg_left,size);
	cv::resize(seg_right, seg_right,size);

	while (nh.ok()) {
		ros::spinOnce();
		freshJoints = davinci_interface::get_last_robot_pos(joints);
		
		for(int i = 0; i < 7; i++){
			velocities[0][i] = cmds[0][i] - cmds_prev[0][i];
			velocities[1][i] = cmds[1][i] - cmds_prev[1][i];
			cmds_prev[0][i] = cmds[0][i];
			cmds_prev[1][i] = cmds[1][i];
		}
		
		//ROS_WARN("Vels: %f, %f, %f, %f, %f, %f, %f",velocities[0][0],velocities[0][1],velocities[0][2],velocities[0][3],velocities[0][4],velocities[0][5],velocities[0][6]);
		
		/*** make sure camera information is ready ***/
//		if(!freshCameraInfo)
//		{
//			ROS_INFO("---- inside get cam info -----");
//			//retrive camera info
//			P_l = cameraInfoObj.getLeftProjectionMatrix();
//			P_r = cameraInfoObj.getRightProjectionMatrix();
//
//			if(P_l.at<double>(0,0) != 0 && P_r.at<double>(0,0) != 0)
//			{
//				ROS_INFO("obtained camera info");
//				freshCameraInfo = true;
//			}
//
//		}

		/*** if camera is ready, doing the tracking based on segemented image***/
		//if (freshImage /*&& freshVelocity && freshCameraInfo*/) {

			//t = clock();
//			seg_left = segmentation(rawImage_left);  //or use image_vessselness
//			seg_right = segmentation(rawImage_right);
			//t = clock() - t;


			// body velocity
			//Not useful?
			for (int i(0); i < 6; i++) {
				bodyVel.at<double>(i, 0) = Arr[i];
			}
			
			Eigen::Matrix<double, 14, 1> vm;
			Eigen::Matrix<double, 14, 1> sm;
			for(int i = 0; i < 7; i++){
				vm(i) = velocities[0][i];
				vm(i + 7) = velocities[1][i];
				sm(i) = velocities[0][i];
				sm(i + 7) = velocities[1][i];
			}
			trackingImgs = UKF.trackingTool(bodyVel, seg_left, seg_right, P_l, P_r, vm, sm); //with rendered tool and segmented img
//
			cv::imshow("Rendered Image: Left", trackingImgs[0]);
			cv::imshow("Rendered Image: Right", trackingImgs[1]);
			cv::waitKey(50);

			freshImage = false;
			freshVelocity = false;

		//}

		loop_rate.sleep();  //or cv::waitKey(10);
	}

}
