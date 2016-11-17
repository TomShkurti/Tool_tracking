#ifndef TOOL_MODEL_H
#define TOOL_MODEL_H

#include <vector>
#include <stdio.h>
#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <stdlib.h>
#include <math.h>

#include <string>
#include <cstring>


#include <glm/glm.hpp>

#include <glm/vec3.hpp>
#include <glm/vec2.hpp>



class ToolModel{
 public:

		struct toolModel {
			cv::Matx<double,3,1> tvec_cyl;	//cylinder translation vector
			cv::Matx<double,3,1> rvec_cyl;	//cylinder rotation vector

			cv::Matx<double,3,1> tvec_elp;	//ellipse translation vector
			cv::Matx<double,3,1> rvec_elp;	//ellipse rotation vector

			cv::Matx<double,3,1> tvec_grip1;	//gripper1 translation vector
			cv::Matx<double,3,1> rvec_grip1;	//gripper1 rotation vector

			cv::Matx<double,3,1> tvec_grip2;	//gripper2 translation vector
			cv::Matx<double,3,1> rvec_grip2;	//gripper2 rotation vector

			double theta_ellipse;
			double theta_grip_1;
			double theta_grip_2;  //9 DOF



			//built in operator that handles copying the toolModel in efficient way
			toolModel& operator =(const toolModel src){
				this->theta_ellipse = src.theta_ellipse;
				this->theta_grip_1 = src.theta_grip_1;
				this->theta_grip_2 = src.theta_grip_2;

				for(int i(0); i<3; i++){

					this->tvec_cyl(i) = src.tvec_cyl(i);
					this->rvec_cyl(i) = src.rvec_cyl(i);

					this->tvec_elp(i) = src.tvec_elp(i);
					this->rvec_elp(i) = src.rvec_elp(i);

					this->tvec_grip1(i) = src.tvec_grip1(i);
					this->rvec_grip1(i) = src.rvec_grip1(i);

					this->tvec_grip2(i) = src.tvec_grip2(i);
					this->rvec_grip2(i) = src.rvec_grip2(i);


				}
				return *this;
			}

			// toolModel constructor, creates an empty toolModel
			toolModel(void){
				this->theta_ellipse = 0.0;
				this->theta_grip_1 = 0.0;
				this->theta_grip_2 = 0.0;
				for(int i(0); i<3; i++){
					this->tvec_cyl(i) = 0.0;
					this->rvec_cyl(i) = 0.0;

					this->tvec_elp(i) = 0.0;
					this->rvec_elp(i) = 0.0;

					this->tvec_grip1(i) = 0.0;
					this->rvec_grip1(i) = 0.0;	

					this->tvec_grip2(i) = 0.0;
					this->rvec_grip2(i) = 0.0;	

				}
			}

		};   //end struct
        
		//std::vector< std::vector<double> > I;
		glm::mat3 I;

		glm::mat3 roll_mat;
    	glm::mat3 pitch_mat;
    	glm::mat3 yaw_mat;

    	glm::mat3 rotation_mat;

		glm::vec3 q_1;  //initial point for ellipse
		glm::vec3 q_2;  //intial point for girpper 

/******************the model points and vecs********************/
		std::vector< glm::vec3 > body_vertices;
		std::vector< glm::vec3 > ellipse_vertices;
		std::vector< glm::vec3 > griper1_vertices;
		std::vector< glm::vec3 > griper2_vertices;

		std::vector< glm::vec3 > body_Vnormal;    //vertex normal, for the computation of the silhouette
		std::vector< glm::vec3 > ellipse_Vnormal;
		std::vector< glm::vec3 > griper1_Vnormal;
		std::vector< glm::vec3 > griper2_Vnormal;

		std::vector< cv::Point3d > body_Vpts;     ////to save time from converting
		std::vector< cv::Point3d > ellipse_Vpts;
		std::vector< cv::Point3d > griper1_Vpts;
		std::vector< cv::Point3d > griper2_Vpts;

		std::vector< cv::Point3d > body_Npts;    //vertex normal, for the computation of the silhouette
		std::vector< cv::Point3d > ellipse_Npts;
		std::vector< cv::Point3d > griper1_Npts;
		std::vector< cv::Point3d > griper2_Npts;		
/***************cam view points************************/

		std::vector< cv::Point3d > CamBodyPts;     ////to save time from converting
		std::vector< cv::Point3d > CamEllipPts;
		std::vector< cv::Point3d > CamGripPts_1;
		std::vector< cv::Point3d > CamGripPts_2;

		std::vector< cv::Point3d > CamBodyNorms;    //vertex normal, for the computation of the silhouette
		std::vector< cv::Point3d > CamEllipNorms;
		std::vector< cv::Point3d > CamGripNorms_1;
		std::vector< cv::Point3d > CamGripNorms_2;		
/********************************************************************/
		std::vector< std::vector<int> > body_faces;
		std::vector< std::vector<int> > ellipse_faces;
		std::vector< std::vector<int> > griper1_faces;
		std::vector< std::vector<int> > griper2_faces;

		std::vector< std::vector<int> > body_neighbors;
		std::vector< std::vector<int> > ellipse_neighbors;
		std::vector< std::vector<int> > griper1_neighbors;
		std::vector< std::vector<int> > griper2_neighbors;

		cv::Mat CamMat;

		double  offset_ellipse; //inch
        double offset_gripper; //inch;

        int cyl_size;
        int elp_size;
        int girp1_size;
        int girp2_size;




 		ToolModel(cv::Mat& CamMat);  //cotructor

 		double randomNumber(double stdev, double mean);

 		void load_model_vertices(const char * path, std::vector< glm::vec3 > &out_vertices, std::vector< glm::vec3 > &vertex_normal, 
                                 std::vector< std::vector<int> > &out_faces,  std::vector< std::vector<int> > &neighbor_faces );

 		void modify_model_();  ///there are offsets when loading the model convert from glm to cv

 		toolModel setRandomConfig(const toolModel &initial, double stdev, double mean);

 		void Convert_glTocv_pts(std::vector< glm::vec3 > &input_vertices, std::vector< cv::Point3d > &out_vertices);

 		cv::Point3d Convert_glTocv_pt(glm::vec3 &input_vertex);

 		void ConvertInchtoMeters(std::vector< cv::Point3d > &input_vertices);

 		//cam view need to be modified
 		cv::Rect renderTool(cv::Mat &image, const toolModel &tool, const cv::Mat &P, cv::OutputArray = cv::noArray() );

 		void Compute_Silhouette( const std::vector< std::vector<int> > &input_faces, const std::vector< std::vector<int> > &neighbor_faces, 
                                 const std::vector< cv::Point3d > &input_vertices, const  std::vector< cv::Point3d > &input_Vnormal,
                                   cv::Mat &image, const cv::Mat &rvec, const cv::Mat &tvec, const cv::Mat &P, cv::OutputArray jac, cv::Point2d &XY_max, cv::Point2d &XY_min);

 		cv::Point3d FindFaceNormal(cv::Point3d &input_v1, cv::Point3d &input_v2, cv::Point3d &input_v3,
                                     cv::Point3d &input_n1, cv::Point3d &input_n2, cv::Point3d &input_n3);

 		int Compare_vertex(std::vector<int> &vec1, std::vector<int> &vec2);

 		cv::Point3d transformation_nrms(const cv::Point3d &vec, const cv::Mat& rvec, const cv::Mat &tvec);
 		cv::Point3d transformation_pts(const cv::Point3d &point, const cv::Mat& rvec, const cv::Mat &tvec);

 		cv::Point3d crossProduct(cv::Point3d &vec1, cv::Point3d &vec2);
 		double dotProduct(cv::Point3d &vec1, cv::Point3d &vec2);
 		cv::Point3d Normalize(cv::Point3d &vec1);
 		cv::Point3d ConvertCelitoMeters(cv::Point3d &input_pt);

 		void camTransformPoints(cv::Mat &cam_mat, std::vector< cv::Point3d > &input_vertices, std::vector< cv::Point3d > &output_vertices);
 		void camTransformVecs(cv::Mat &cam_mat, std::vector< cv::Point3d > &input_normals, std::vector< cv::Point3d > &output_normals);

        // double calculateMatchingScore(cv::Mat &toolImage, const cv::Mat &segmentedImage, cv::Rect &ROI, bool displayPause);

 

};

#endif