#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include "sophus/se3.h"
#include "sophus/so3.h"

#include <opencv2/core/eigen.hpp>

#include "line_extractor.hpp"

#include <iostream>

cv::Mat eigen2mat(Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> A)
{
	cv::Mat B;
	cv::eigen2cv(A,B);
	
	return B;
}

CalibParams* camera_new(string name,
    double fu,
    double fv,
    double cu,
    double cv,
    double dist_coeffs_0,
    double dist_coeffs_1,
    double dist_coeffs_2,
    double dist_coeffs_3,
    double camera_x,
    double camera_y,
    double camera_z,
    double pitch,
    double yaw,
    double roll) {
    CalibParams* camera = new CalibParams;
    camera->fu = fu;
    camera->fv = fv;
    camera->cu = cu;
    camera->cv = cv;
    //camera->K = (Mat_<double>(3,3)<<fu, 0.0, cu, 0.0, fv, cv, 0.0, 0.0, 1.0);
    //camera->D = (Mat_<double>(4,1)<<dist_coeffs_0, dist_coeffs_1, dist_coeffs_2, dist_coeffs_3);
    camera->dist_coeffs_0 = dist_coeffs_0;
    camera->dist_coeffs_1 = dist_coeffs_1;
    camera->dist_coeffs_2 = dist_coeffs_2;
    camera->dist_coeffs_3 = dist_coeffs_3;

    camera->camera_x = camera_x;
    camera->camera_y = camera_y;
    camera->camera_z = camera_z;
    //camera->tvecs = (Mat_<double>(3,1)<<camera_x, camera_y, camera_z);


    camera->pitch = pitch;
    camera->yaw = yaw;
    camera->roll = roll;
    return camera;
}

void print_calib_params(CalibParams* camera) {
    std::cout << "camera->fu:" << camera->fu << "\n";
    std::cout << "camera->fv:" << camera->fv << "\n";
    std::cout << "camera->cu:" << camera->cu << "\n";
    std::cout << "camera->cv:" << camera->cv << "\n";
    std::cout << "camera->dist_coeffs_0:" << camera->dist_coeffs_0 << "\n";
    std::cout << "camera->dist_coeffs_1:" << camera->dist_coeffs_1 << "\n";
    std::cout << "camera->dist_coeffs_2:" << camera->dist_coeffs_2 << "\n";
    std::cout << "camera->dist_coeffs_3:" << camera->dist_coeffs_3 << "\n";
    std::cout << "camera->camera_x:" << camera->camera_x << "\n";
    std::cout << "camera->camera_y:" << camera->camera_y << "\n";
    std::cout << "camera->camera_z:" << camera->camera_z << "\n";
    //std::cout << "camera->K:\n" << camera->K << "\n";
    //std::cout << "camera->D:\n" << camera->D << "\n";
    //std::cout << "camera->tvecs:\n" << camera->tvecs << "\n";

    std::cout << "camera->pitch:" << camera->pitch << "\n";
    std::cout << "camera->yaw:" << camera->yaw << "\n";
    std::cout << "camera->roll:" << camera->roll << "\n";
}

one_frame_lines_set* one_frame_set_new() {
    one_frame_lines_set* lines = new one_frame_lines_set;
    return lines;
}

void init_intrinsic(CalibParams* camera, Mat* cam_matrix, Mat* dist_coeffs) {
    *cam_matrix = (Mat_<double>(3,3)<<camera->fu, 0.0, camera->cu, 0.0, camera->fv, camera->cv, 0.0, 0.0, 1.0);
    *dist_coeffs = (Mat_<double>(4,1)<<camera->dist_coeffs_0, camera->dist_coeffs_1, camera->dist_coeffs_2, camera->dist_coeffs_3);
}

cv::Mat get_extrinsic_mat(CalibParams* camera) {
    //for camera, pitch ( around x ) , yaw ( around y ) ,  roll(around z )
    //camera coordinate:
    //camera center: O(0,0,0)
    //camera LookAt: +Z
    //camera Right:  +X
    //camera Left:   +Y
    //around the X axis: pitch
    //around the Y axis: yaw
    //around the Z axis: roll
    cv::Vec3f theta = 
    //cv::Vec3f(angle_to_radian(0), 0, 0);//if not rotate, the result is fisheye undistorted?
    //cv::Vec3f(angle_to_radian(45), 0, 0);
    //cv::Vec3f(angle_to_radian(45), 0, angle_to_radian(-90));
    cv::Vec3f(angle_to_radian(camera->pitch), angle_to_radian(camera->yaw), angle_to_radian(camera->roll));
    cv::Mat mat_R = eulerAnglesToRotationMatrix(theta);
    // 计算得到extrinsic_matrix_
    //cv::Mat euler = 
    //(cv::Mat_<double>(3, 1) << angle_to_radian(45), 0, 0);//front
    //(cv::Mat_<double>(3, 1) << (45.0f * M_PI)/180, 0, (45.0f * M_PI)/180);//left
    //(cv::Mat_<double>(3, 1) << angle_to_radian(0), 0, angle_to_radian(0));
    //(cv::Mat_<double>(3, 1) << angle_to_radian(camera->pitch), angle_to_radian(camera->yaw), angle_to_radian(camera->roll));
    //(cv::Mat_<double>(3, 1) << extrinsic.pitch, extrinsic.yaw, extrinsic.roll);


    // cout << "euler:\n" << euler << endl;
    // cv::Mat mat_R;
    // cv::Rodrigues(euler, mat_R, cv::noArray());//not what i understand


    // cout << "delia test:\n";
    // cout << "mat_R:\n" << mat_R << endl;//3.1367766857147217, -0.0018368728924542665, 1.049095630645752
    //cv::Mat mat_T =
    //(cv::Mat_<double>(3, 1) << 0,0,-1.04);//move fisheye camera
    //(cv::Mat_<double>(3, 1) << 0,-3.13,-1.04);//front
    //(cv::Mat_<double>(3, 1) << 1.19,0,-1.04);//left
    //(cv::Mat_<double>(3, 1) << 0,3.13,-1.04);//rear
    //(cv::Mat_<double>(3, 1) << -1.19,0,-1.04);//right
    //(cv::Mat_<double>(3, 1) << 3.1367766857147217, -0.0018368728924542665, 1.049095630645752);//front
    //     //camera->tvecs;
    //     //(cv::Mat_<double>(3, 1) << extrinsic_.X, extrinsic_.Y, extrinsic_.Z);
    //cout << "mat_T:\n" << mat_T << endl;
    

    theta  = cv::Vec3f(angle_to_radian(180), 0, angle_to_radian(90));
    cv::Mat mat_R2 = eulerAnglesToRotationMatrix(theta);
    //cv::Rodrigues(euler2, mat_R2, cv::noArray());
    cv::Mat mat_T2 = 10*(cv::Mat_<double>(3, 1) << camera->camera_x, camera->camera_y, camera->camera_z);
    cout << "mat_T2:\n" << mat_T2 << endl;
    //mat_T2 = mat_R2*mat_T2;
    //cout << "mat_R2*mat_T2:\n" << mat_T2 << endl;

    // cv::Mat mat_T3 = (cv::Mat_<double>(3, 1) << 0-camera->tvecs.at<double>(1), 0-camera->tvecs.at<double>(0), 0-camera->tvecs.at<double>(2));
    // cout << "mat_T3:\n" << mat_T3 << endl;

    cv::Mat mat_RT;
    cv::hconcat(mat_R, mat_T2, mat_RT);
    cout << "mat_RT:\n" << mat_RT << endl;
    //[R T; 0 1]
    cv::Mat mat_01 = (cv::Mat_<double>(1, 4) << 0, 0, 0, 1);
    cv::Mat mat_RT01;
    cv::vconcat(mat_RT, mat_01, mat_RT01);
    cout << "mat_RT01:\n" << mat_RT01 << endl;
    cv::Mat mat_RT01_inv;
    cv::invert(mat_RT01, mat_RT01_inv);
    return mat_RT01_inv;//mat_RT01.inv();
}


void undistortFisheye(const cv::Mat &image_in, cv::Mat camera_intrinsic_matrix, cv::Vec4d distort_coeff_matrix, cv::Mat camera_extrinsic_matrix, cv::Mat &image_out) {
    double dx = 0.01;//unit:m/pixel
    double dy = 0.01;
    double scale = 3.6;
    double fx = 1/dx*scale;
    double fy = 1/dy*scale;
    double cx = avm_w/2;
    double cy = avm_h/2;
    cv::Mat KG = (Mat_<double>(3,3)<<fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0);
    cv::Mat DG = (Mat_<double>(4,1)<<0, 0, 0, 0);
    cv::Size img_size = cv::Size(avm_w, avm_h);//image_in.size();

    cv::Vec3f theta = cv::Vec3f(angle_to_radian(0), angle_to_radian(0), angle_to_radian(0));
    cv::Mat mat_R = eulerAnglesToRotationMatrix(theta);

    cv::Vec3f theta_update = cv::Vec3f(angle_to_radian(0), angle_to_radian(0), angle_to_radian(0));
    mat_R = eulerAnglesToRotationMatrix(theta_update)*eulerAnglesToRotationMatrix(theta);

    cv::Mat newCamMat;
    cv::fisheye::estimateNewCameraMatrixForUndistortRectify(
        //camera_intrinsic_matrix, distort_coeff_matrix, img_size,
        KG, DG, img_size,
        cv::Matx33d::eye(),//cv::Mat::eye(3,3,CV_32FC1) 
        newCamMat, 1);//

    cv::Mat map1, map2;
    cv::fisheye::initUndistortRectifyMap(
            camera_intrinsic_matrix, 
            distort_coeff_matrix, 
            mat_R,//cv::Mat::eye(3,3,CV_32FC1),//cv::Matx33d::eye()
            newCamMat,//camera_intrinsic_matrix,//
            img_size,
            CV_32FC1, map1, map2);

    cv::Mat correct_image;
    cv::remap(image_in, image_out, map1, map2, cv::INTER_LINEAR);

}



void undistortFisheye(const cv::Mat &image_in, cv::Mat camera_intrinsic_matrix, cv::Vec4d distort_coeff_matrix, cv::Mat &image_out) {
    // TODO: can do better, pinhole model -> ocam model?


    // we should know whether camera_intrinsic_matrix need be update
    // by cv::fisheye::estimateNewCameraMatrixForUndistortRectify or not
    cv::Size img_size = image_in.size();
    cv::Mat newCamMat;
    cv::fisheye::estimateNewCameraMatrixForUndistortRectify(
        camera_intrinsic_matrix, distort_coeff_matrix, img_size,
        cv::Mat::eye(3,3,CV_32FC1), newCamMat, 1);//cv::Matx33d::eye()

    // cv::Mat newCamMat = camera_intrinsic_matrix;
    // newCamMat.at<float>(0,2) = image_in.cols/2;
    // newCamMat.at<float>(1,2) = image_in.rows/2;

    cv::Mat map1, map2;
    cv::fisheye::initUndistortRectifyMap(
            camera_intrinsic_matrix, 
            distort_coeff_matrix, 
            cv::Mat::eye(3,3,CV_32FC1),//cv::Matx33d::eye()
            newCamMat,//camera_intrinsic_matrix,//
            img_size,
            CV_32FC1, map1, map2);

    cv::Mat correct_image;
    cv::remap(image_in, image_out, map1, map2, cv::INTER_LINEAR);
cout << "newCamMat:" << newCamMat << endl;
//cv::imshow("image_out.png", image_out);
//cv::imwrite("image_out.png", image_out);
//cv::waitKey(0);
}

cv::Mat
CreateBirdeyeView(const cv::Mat &img,
               const CameraRotationEuler &rotation_angle,
               const CameraIntrinsic &intrinsic,
               const float &camera_height) {
  cv::Mat bv = cv::Mat::zeros(940, 700, CV_8UC1);
  cv::Mat Twc = cv::Mat::zeros(4, 4, CV_32F);
  Twc.at<float>(0, 3) = 0;
  Twc.at<float>(1, 3) = -camera_height;
  Twc.at<float>(2, 3) = 0;
  Twc.at<float>(3, 3) = 1.0;
  Eigen::Matrix3f Rx =
      (Eigen::AngleAxisf(rotation_angle.pitch, Eigen::Vector3f::UnitX()))
          .matrix();
  Eigen::Matrix3f Ry =
      (Eigen::AngleAxisf(-rotation_angle.yaw, Eigen::Vector3f::UnitY()))
          .matrix();
  Eigen::Matrix3f Rz =
      (Eigen::AngleAxisf(rotation_angle.roll, Eigen::Vector3f::UnitZ()))
          .matrix();
  Eigen::Matrix3f R = Rx * Rz * Ry;

  cv::Mat Rcw = (cv::Mat_<float>(3, 3) << R(0, 0), R(0, 1), R(0, 2), R(1, 0),
                 R(1, 1), R(1, 2), R(2, 0), R(2, 1), R(2, 2));

  cv::Mat Rwc = Rcw.inv();
  Rwc.copyTo(Twc.rowRange(0, 3).colRange(0, 3));
  cv::Mat Tcw = Twc.inv();
  for (int i = 0; i < bv.rows; i++) {
    for (int j = 0; j < bv.cols; j++) {
      float x = float((j - 350) * 0.03);
      float y = 0;
      float z = float(40 - 0.03 * i);

      cv::Mat wp = (cv::Mat_<float>(3, 1) << x, y, z);
      cv::Mat camp =
          Tcw.rowRange(0, 3).colRange(0, 3) * wp + Tcw.rowRange(0, 3).col(3);

      float camx = camp.at<float>(0, 0);
      float camy = camp.at<float>(1, 0);
      float camz = camp.at<float>(2, 0);

      float u = (camx / camz) * intrinsic.fx + intrinsic.cx;
      float v = (camy / camz) * intrinsic.fy + intrinsic.cy;

      int rows = int(v);
      int cols = int(u);
      if (rows < 0 || rows >= img.rows)
        continue;
      if (cols < 0 || cols >= img.cols)
        continue;
      bv.at<uchar>(i, j) = img.at<uchar>(rows, cols);
    }
  }
  cv::line(bv, cv::Point2f(10, 0), cv::Point2f(10, 940), cv::Scalar(255), 1);
  cv::line(bv, cv::Point2f(110, 0), cv::Point2f(110, 940), cv::Scalar(255), 1);
  cv::line(bv, cv::Point2f(210, 0), cv::Point2f(210, 940), cv::Scalar(255), 1);
  cv::line(bv, cv::Point2f(310, 0), cv::Point2f(310, 940), cv::Scalar(255), 1);
  cv::line(bv, cv::Point2f(410, 0), cv::Point2f(410, 940), cv::Scalar(255), 1);
  cv::line(bv, cv::Point2f(510, 0), cv::Point2f(510, 940), cv::Scalar(255), 1);
  cv::line(bv, cv::Point2f(610, 0), cv::Point2f(610, 940), cv::Scalar(255), 1);
  cv::line(bv, cv::Point2f(710, 0), cv::Point2f(710, 940), cv::Scalar(255), 1);
  cv::line(bv, cv::Point2f(810, 0), cv::Point2f(810, 940), cv::Scalar(255), 1);

  return bv;
}



int CalculateRawPitch(const CameraIntrinsic &intrinsic,
                      std::vector<LaneCoef> &lane_data,
                      cv::Point2f &vanish_point, float &pitch_raw) {
  if (lane_data.size() != 2) {
    std::cout << "not input two lane data \n";
    return 1;
  }

  float k1 = lane_data[0].coef_k;
  float b1 = lane_data[0].coef_b;
  float k2 = lane_data[1].coef_k;
  float b2 = lane_data[1].coef_b;

  // calculate raw_pitch
  // note the k1 k2 will not be nan
  // intersetion point--vanishing point
  float x = -(b1 - b2) / (k1 - k2);
  float y = k1 * x + b1;
  vanish_point.x = x;
  vanish_point.y = y;
  // calculate homo point
  float cam_z = 1;
  float cam_y = (y - cam_z * intrinsic.cy) / intrinsic.fy;
  cv::Mat ground_inter = (cv::Mat_<float>(3, 1) << 0, 0, 1);
  cv::Mat camera_inter = (cv::Mat_<float>(3, 1) << 0, cam_y, cam_z);
  camera_inter = camera_inter / cv::norm(camera_inter);
  cv::Mat rotation = ground_inter.cross(camera_inter);

  float sina = float(cv::norm(rotation));
  pitch_raw = -asin(sina);
  std::cout << "\033[1;31m"
            << " raw pitch " << pitch_raw << "\033[0m" << std::endl;

  return 0;
}

void oneview_extract_line(cv::Mat *img, cv::Mat *birdeye_img, CalibParams* camera, CalibParams* camera_v)
{
    cv::Mat camera_intrinsic;
    cv::Mat camera_dist_coeffs;
    init_intrinsic(camera, &camera_intrinsic, &camera_dist_coeffs);

    cv::Mat image_undistort;
    undistortFisheye(*img, camera_intrinsic, camera_dist_coeffs, image_undistort);
    //undistortFisheye(*img, camera_intrinsic, camera_dist_coeffs, get_extrinsic_mat(camera), image_undistort);

    cv::Mat image_undistort_gray;
    cv::cvtColor(image_undistort, image_undistort_gray, COLOR_BGR2GRAY);


    std::vector<cv::Point2f> point_l;
    std::vector<cv::Point2f> point_r;
    RoiData roi_data;
    roi_data.width_top = 300;
    roi_data.width_bottom = 1000;
    roi_data.height_min = 325;
    roi_data.height_max = 475;
//cv::rectangle(image_undistort, cv::Rect(roi_data.width_top, roi_data.height_min, roi_data.width_bottom-roi_data.width_top, roi_data.height_max-roi_data.height_min), cv::Scalar(255, 0, 0), 1);
cv::imshow("image_undistort.png", image_undistort);
cv::imwrite("image_undistort.png", image_undistort);
cv::waitKey(0);

    CameraIntrinsic intrinsic;
    intrinsic.fx = 103.5691546824274;//camera->K.at<double>(0,0);
    intrinsic.fy = 103.569;//camera->K.at<double>(1,1);
    intrinsic.cx = 640;//camera->K.at<double>(0,2);
    intrinsic.cy = 360;//camera->K.at<double>(1,2);

    int canny_threshold = 300;//50 100 150 200 250
    detect_line(image_undistort_gray,
                roi_data,
                point_l,
                point_r,
                canny_threshold);
//ps: point_l.y is col number; point_l.x is row number; i have change it
//ps: point_r.y is col number; point_r.x is row number; i have change it
    // for (auto p : point_l)
    // {
    //   cv::circle(image_undistort, p, 2, cv::Scalar(0, 0, 255), 2, CV_AA);
    //   //cv::Point(p.y, p.x)
    // }

    // for (auto p : point_r)
    // {
    //   cv::circle(image_undistort, p, 2, cv::Scalar(0, 255, 0), 2, CV_AA);
    // }

cv::imshow("image_undistort.png", image_undistort);
//cv::imwrite("image_undistort.png", image_undistort);
cv::waitKey(0);

    // lane_kb : save two lane marking k and b
    float lane_kb[4] = {0.0f};
    // lane_kb[0]: k of left lane
    // lane_kb[1]: b of left lane
    // lane_kb[2]: k of right lane
    // lane_kb[3]: b of right lane
    
    cv::Vec4f line_infomation;
    float k = 0.0, b = 0.0;
    fitLineRansac(point_l, line_infomation, 1000, 10);

    k = line_infomation[1] / line_infomation[0];
    b = line_infomation[3] - k * line_infomation[2];
    lane_kb[0] = k;
    lane_kb[1] = b;

    fitLineRansac(point_r, line_infomation, 1000, 10);
    k = line_infomation[1] / line_infomation[0];
    b = line_infomation[3] - k * line_infomation[2];
    lane_kb[2] = k;
    lane_kb[3] = b;


    std::vector<LaneCoef> lane_coefs_each_image;
    std::cout << "k1: " << lane_kb[0] << "  b1: " << lane_kb[1] << std::endl;
    std::cout << "k2: " << lane_kb[2] << "  b2: " << lane_kb[3] << std::endl;
    LaneCoef lane_coef_left;
    LaneCoef lane_coef_right;
    lane_coef_left.valid = true;
    lane_coef_left.coef_k = lane_kb[0];
    lane_coef_left.coef_b = lane_kb[1];
    lane_coef_right.valid = true;
    lane_coef_right.coef_k = lane_kb[2];
    lane_coef_right.coef_b = lane_kb[3];
    lane_coefs_each_image.push_back(lane_coef_left);
    lane_coefs_each_image.push_back(lane_coef_right);   

//draw lane
//y is row number, x is col number
    for (size_t j = 0; j < lane_coefs_each_image.size(); j++) {
        const int ky0 = 325;
        const int ky1 = 475;
        float x0 = (ky0 - lane_coefs_each_image[j].coef_b) / lane_coefs_each_image[j].coef_k;
        cv::Point2f pt5(x0, ky0);
        float x1 = (ky1 - lane_coefs_each_image[j].coef_b) / lane_coefs_each_image[j].coef_k;
        cv::Point2f pt6(x1, ky1);
        cv::line(image_undistort, pt5, pt6, cv::Scalar(255, 255, 0), 1);
    }
cv::imshow("image_undistort_lane.png", image_undistort);
cv::imwrite("image_undistort_lane.png", image_undistort);
cv::waitKey(0);


    // return vanishing point-inter
    cv::Point2f inter;
    float pitch_raw;

    CalculateRawPitch(intrinsic, lane_coefs_each_image, inter, pitch_raw);
    cout << "pitch_raw=" << pitch_raw << endl;
    cout << "inter=" << inter << endl;
    cv::circle(image_undistort, inter, 2, cv::Scalar(0, 0, 255), 2, CV_AA);

cv::imshow("image_undistort_lane_inter.png", image_undistort);
cv::imwrite("image_undistort_lane_inter.png", image_undistort);
cv::waitKey(0);


}

void oneview_extract_line(cv::Mat *img, cv::Mat *birdeye_img, CalibParams* camera, CalibParams* camera_v, one_frame_lines_set* res, vanishing_pts* v_pts) {
    birdeye_oneview_transform(img, birdeye_img, camera, camera_v);

    // //convert bgr to gray
    // cv::Mat birdeye_gray;
    // cv::cvtColor(*birdeye_img, birdeye_gray, COLOR_BGR2GRAY);


    //Extracts lanemarks






    //to do:
    //save res and v_pts
    //res->orig_lines = ;
    //v_pts->front_vp = inter;

}

cv::Mat ground_stitch(cv::Mat img_GF, cv::Mat img_GL,
	cv::Mat img_GB, cv::Mat img_GR,
	int rows, int cols)
{

	cv::Mat img_G(rows, cols, CV_8UC3);
	for (int i = 0;i < rows;i++)
	{
		for (int j = 0;j < cols;j++)
		{
			if (i > 800)
			{

				img_G.at<cv::Vec3b>(i, j) = img_GB.at<cv::Vec3b>(i, j);
			}
			else if (i < 200)
			{
				img_G.at<cv::Vec3b>(i, j) = img_GF.at<cv::Vec3b>(i, j);
			}
			else
			{
				if (j < 400)
				{
					img_G.at<cv::Vec3b>(i, j) = img_GL.at<cv::Vec3b>(i, j);
				}
				else if (j > 600)
				{
					img_G.at<cv::Vec3b>(i, j) = img_GR.at<cv::Vec3b>(i, j);
				}
				else {
					//here is car
					img_G.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 0, 0);
				}
			}

		}
	}


	return img_G;
}

//T_CG: ??
// cv::Mat project_on_ground(cv::Mat img, cv::Mat T_CG, cv::Mat K_C, cv::Mat D_C, cv::Mat K_G, int rows, int cols) {
//     // 	cout<<"--------------------Init p_G and P_G------------------------"<<endl;
//     //For a point pG = [uG, vG, 1]T in the bird’s - eye - view image
//     cv::Mat p_G = cv::Mat::ones(3, rows * cols, CV_64FC1);
//     for (int i = 0;i < rows;i++)
//     {
//         for (int j = 0;j < cols;j++)
//         {
//             p_G.at<double>(0, cols * i + j) = j;//x, col number
//             p_G.at<double>(1, cols * i + j) = i;//y, row number
//             //p_G.at<double>(2, cols * i + j) = 1;//1
//         }
//     }
//     //cout << "p_G: " << endl;
//     //cout << p_G(cv::Rect(0, 0, 100, 3)) << endl << endl;


//     //For a point, PG = [XG, YG, ZG, 1]T in OG, ZG = 0
//     cv::Mat P_G = cv::Mat::ones(4, rows * cols, CV_64FC1);
//     P_G(cv::Rect(0, 0, rows * cols, 3)) = K_G.inv() * p_G;//pG = KG*PG
//     P_G(cv::Rect(0, 2, rows * cols, 1)) = 0;//ZG = 0
//     //P_G(cv::Rect(0, 3, rows * cols, 1)) = 1;//1

//     // 	cout<<"--------------------Init P_GC------------------------"<<endl;
//     //cout << "P_GC: " << endl;
//     //cout << P_G(cv::Rect(0, 0, 100, 4)) << endl << endl;

//     // the pixel coordinate pCi of PG in the ith camera Ci
//     cv::Mat P_GC = cv::Mat::zeros(4, rows * cols, CV_64FC1);
//     cout << "T_CG:" << T_CG << endl;
//     P_GC = T_CG * P_G;//.t() 
//     //cout << "P_GC: " << endl;
//     //cout << P_GC(cv::Rect(0, 0, 100, 4)) << endl << endl;
//     //cout << P_GC(cv::Rect(0, 2, 100, 2)) << endl << endl;

//     // 	cout<<"--------------------Init P_GC1------------------------"<<endl;
//     cv::Mat P_GC1 = cv::Mat::zeros(1, rows * cols, CV_64FC2);//???
//     vector<cv::Mat> channels;
//     //channels.resize(2);
//     //cv::Mat channels[2];
//     cv::split(P_GC1, channels);
//     channels[0] = P_GC(cv::Rect(0, 0, rows * cols, 1)) / P_GC(cv::Rect(0, 2, rows * cols, 1));
//     channels[1] = P_GC(cv::Rect(0, 1, rows * cols, 1)) / P_GC(cv::Rect(0, 2, rows * cols, 1));
//     cv::merge(channels, P_GC1);
//     //cout << "P_GC1: " << endl;
//     //cout << P_GC1(cv::Rect(0, 0, 5, 1)) << endl << endl;

//     // 	cout<<"--------------------Init p_GC------------------------"<<endl;
//     //For a point pGC = [uGC, vGC, 1]T in the fish eye - view image
//     cv::Mat p_GC = cv::Mat::zeros(1, rows*cols, CV_64FC2);//???rows * cols

//     vector<double> D_C_{ D_C.at<double>(0),D_C.at<double>(1),D_C.at<double>(2),D_C.at<double>(3) };
//     cv::fisheye::distortPoints(P_GC1, p_GC, K_C, D_C_);
//     //cout << "p_GC: " << endl;
//     p_GC.reshape(rows, cols);
//     cv::Mat p_GC_table = p_GC.reshape(0, rows);
//     //vector<cv::Mat> p_GC_table_channels(2);
//     cv::Mat p_GC_table_channels[2];
//     cv::split(p_GC_table, p_GC_table_channels);

//     cv::Mat p_GC_table_32F;
//     p_GC_table.convertTo(p_GC_table_32F, CV_32FC2);

//     cv::Mat img_GC;
//     cv::remap(img, img_GC, p_GC_table_32F, cv::Mat(), cv::INTER_LINEAR);
//     // 	img_GC = bilinear_interpolation(img,p_GC_table,rows,cols);
//     // 	cout<<img_GC.size<<endl;

//     return img_GC;
// }


cv::Mat project_on_ground(cv::Mat img, Sophus::SE3 T_CG,
                        Eigen::Matrix3d K_C,Eigen::Vector4d D_C,
                        cv::Mat K_G,int rows, int cols)
{
    // 	cout<<"--------------------Init p_G and P_G------------------------"<<endl;
    cv::Mat p_G = cv::Mat::ones(3,rows*cols,CV_64FC1);
    for(int i=0;i<rows;i++)
    {
        for(int j=0;j<cols;j++)
        {
            p_G.at<double>(0,cols*i+j) = j;
            p_G.at<double>(1,cols*i+j) = i;
        }
    }

    cv::Mat P_G = cv::Mat::ones(4,rows*cols,CV_64FC1);
    P_G(cv::Rect(0,0,rows*cols,3)) = K_G.inv()*p_G;
    P_G(cv::Rect(0,2,rows*cols,1)) = 0;

    // 	cout<<"--------------------Init P_GF------------------------"<<endl;

    cv::Mat P_GC = cv::Mat::zeros(4,rows*cols,CV_64FC1);
    cv::Mat T_CG_(4,4,CV_64FC1);
    cv::eigen2cv(T_CG.matrix(),T_CG_);
    cout << "T_CG_:" << T_CG_ << endl;
    P_GC =  T_CG_ * P_G;


    cv::Mat P_GC1 = cv::Mat::zeros(1,rows*cols,CV_64FC2);
    vector<cv::Mat> channels(2);
    cv::split(P_GC1, channels);
    channels[0] = P_GC(cv::Rect(0,0,rows*cols,1))/P_GC(cv::Rect(0,2,rows*cols,1));
    channels[1] = P_GC(cv::Rect(0,1,rows*cols,1))/P_GC(cv::Rect(0,2,rows*cols,1));
    cv::merge(channels, P_GC1);

    cv::Mat p_GC = cv::Mat::zeros(1,rows*cols,CV_64FC2);
    // 	cout<<eigen2mat(K_C)<<endl;
    vector<double> D_C_{D_C(0,0),D_C(1,0),D_C(2,0),D_C(3,0)};
    cv::fisheye::distortPoints(P_GC1,p_GC,eigen2mat(K_C),D_C_);
    // 	cout<<"p_GC: "<<endl;
    p_GC.reshape(rows,cols);
    cv::Mat p_GC_table = p_GC.reshape(0,rows);
    vector<cv::Mat> p_GC_table_channels(2);
    cv::split(p_GC_table, p_GC_table_channels);

    cv::Mat p_GC_table_32F;
    p_GC_table.convertTo(p_GC_table_32F,CV_32FC2);

    cv::Mat img_GC;
    cv::remap(img,img_GC,p_GC_table_32F,cv::Mat(),cv::INTER_LINEAR);

    return img_GC;
}


// Calculates rotation matrix given euler angles.
Mat eulerAnglesToRotationMatrix(Vec3f &theta)
{
    // Calculate rotation about x axis
    Mat R_x = (Mat_<double>(3,3) <<
               1,       0,               0,
               0,       cos(theta[0]),   -sin(theta[0]),
               0,       sin(theta[0]),   cos(theta[0])
               );

    // Calculate rotation about y axis
    Mat R_y = (Mat_<double>(3,3) <<
            cos(theta[1]),    0,      sin(theta[1]),
            0,                1,      0,
            -sin(theta[1]),   0,      cos(theta[1])
            );
    
    // Calculate rotation about z axis
    Mat R_z = (Mat_<double>(3,3) <<
            cos(theta[2]),    -sin(theta[2]),      0,
            sin(theta[2]),    cos(theta[2]),       0,
            0,                0,                   1);
    
    
    // Combined rotation matrix
    Mat R = R_z * R_y * R_x;
    
    return R;

}



// Checks if a matrix is a valid rotation matrix.
bool isRotationMatrix(Mat &R)
{
    Mat Rt;
    transpose(R, Rt);
    Mat shouldBeIdentity = Rt * R;
    Mat I = Mat::eye(3,3, shouldBeIdentity.type());
    return  norm(I, shouldBeIdentity) < 1e-6;
}

// Calculates rotation matrix to euler angles
// The result is the same as MATLAB except the order
// of the euler angles ( x and z are swapped ).
Vec3f rotationMatrixToEulerAngles(Mat &R)
{

    assert(isRotationMatrix(R));
    
    float sy = sqrt(R.at<double>(0,0) * R.at<double>(0,0) +  R.at<double>(1,0) * R.at<double>(1,0) );

    bool singular = sy < 1e-6; // If

    float x, y, z;
    if (!singular)
    {
        x = atan2(R.at<double>(2,1) , R.at<double>(2,2));
        y = atan2(-R.at<double>(2,0), sy);
        z = atan2(R.at<double>(1,0), R.at<double>(0,0));
    }
    else
    {
        x = atan2(-R.at<double>(1,2), R.at<double>(1,1));
        y = atan2(-R.at<double>(2,0), sy);
        z = 0;
    }
    return Vec3f(x, y, z);

}

//角度转换为弧度
double angle_to_radian(double degree, double min, double second)
{
	double flag = (degree < 0)? -1.0 : 1.0;			//判断正负
    if(degree<0)
    {
        degree = degree * (-1.0);
    }
	double angle = degree + min/60 + second/3600;
	double result =flag * (angle * M_PI)/180;
	return result;
	//cout<<result<<endl;
}
//弧度转换为角度
void radian_to_angle(double rad, double ang[])
{
    double flag = (rad < 0)? -1.0 : 1.0;
    if(rad<0)
    {
		rad = rad * (-1.0);
    }
	double result = (rad*180)/M_PI;			
	double degree = int(result);
	double min =(result - degree)*60;
	double second = (min - int(min)) * 60;	
	ang[0] = flag * degree;
	ang[1] = int(min);
	ang[2] = second;
}


Sophus::SE3 get_TCG(CalibParams* camera) {
    Eigen::Vector3d t_CG(camera->camera_x, camera->camera_y, camera->camera_z);
    //change x and y
    Eigen::AngleAxisd v1(M_PI/2, Eigen::Vector3d::UnitZ());//rotate 90 degrees around z ---> change x and y
    Eigen::AngleAxisd v2(M_PI, Eigen::Vector3d::UnitX());//rotate 180 degrees around x ---> change -y to +y
    t_CG = v2.matrix()*v1.matrix()*t_CG;

    Eigen::Vector3d ea_CG(angle_to_radian(-camera->yaw), angle_to_radian(-camera->roll), angle_to_radian(-camera->pitch));
    Eigen::Matrix3d R_CG = Eigen::Matrix3d();//rotation matrix
    R_CG = Eigen::AngleAxisd(ea_CG[0], Eigen::Vector3d::UnitZ()) *
           Eigen::AngleAxisd(ea_CG[1], Eigen::Vector3d::UnitY()) *
           Eigen::AngleAxisd(ea_CG[2], Eigen::Vector3d::UnitX());

    t_CG = t_CG;
    Sophus::SE3 T_CG = Sophus::SE3(R_CG,t_CG);
    return T_CG.inverse();
}

Eigen::Matrix3d get_KC(CalibParams* camera) {
    Eigen::Matrix3d K_C;
    K_C <<  camera->fu,   0.00000000,   camera->cu,
            0.00000000,   camera->fv,   camera->cv,
            0.00000000,   0.00000000,   1.00000000;
    return K_C;
}

cv::Mat get_KC_Mat(CalibParams* camera) {
    cv::Mat K_C = (Mat_<double>(3,3) <<
            camera->fu,   0.00000000,   camera->cu,
            0.00000000,   camera->fv,   camera->cv,
            0.00000000,   0.00000000,   1.00000000);
    return K_C;
}

Eigen::Vector4d get_DC(CalibParams* camera) {
    Eigen::Vector4d K_D;
    K_D <<  camera->dist_coeffs_0, camera->dist_coeffs_1, camera->dist_coeffs_2, camera->dist_coeffs_3;
    return K_D;
}


void birdeye_oneview_transform(cv::Mat *img, cv::Mat *birdeye_img, CalibParams* camera, CalibParams* camera_v) {
cv::imshow("img.png", *img);
cv::waitKey(0);


    // CameraRotationEuler rotation_angle_raw;
    // rotation_angle_raw.pitch = 0 * CV_PI / 180.0;
    // rotation_angle_raw.yaw = 0 * CV_PI / 180.0;
    // rotation_angle_raw.roll = 0 * CV_PI / 180.0;
    // float camera_height = 1.049;
    // cv::Mat image_birdeye = CreateBirdeyeView(image_undistort_gray,
    //            rotation_angle_raw,
    //            intrinsic,
    //            camera_height) ;
    // cv::imshow("image_birdeye.png", image_birdeye);
    // cv::waitKey(0);

    //cv::Mat camera_T = get_extrinsic_mat(camera);


    // cv::Mat mat_T = (cv::Mat_<double>(4, 4) << 
    // 1, 0, 0, 0,
    // 0, 1, 0, 0,
    // 0, 0, 1, 1,
    // 0, 0, 0, 1
    // );
    // cv::Vec3f theta = cv::Vec3f(angle_to_radian(45), 0, 0);
    // cv::Mat mat_R = eulerAnglesToRotationMatrix(theta);
    // cv::Mat mat_00 = (cv::Mat_<double>(3, 1) << 0, 0, 0);
    // cv::hconcat(mat_R, mat_00, mat_R);
    // cv::Mat mat_01 = (cv::Mat_<double>(1, 4) << 0, 0, 0, 1);
    // cv::vconcat(mat_R, mat_01, mat_R);
    // cv::Mat homography = camera_v->K * ( mat_T * (mat_R * camera->K));
    // warpPerspective(*img, *birdeye_img, homography, cv::Size(avm_w, avm_h));

    *birdeye_img = 
    //project_on_ground(*img, camera_T, camera->K, camera->D, camera_v->K, avm_h, avm_w);
    project_on_ground(*img, get_TCG(camera), get_KC(camera), get_DC(camera), get_KC_Mat(camera_v), avm_h, avm_w);
    //cv::circle(*birdeye_img, cv::Point2f(avm_w/2, avm_h/2), 1, cv::Scalar(255, 0, 255), 1);
    cv::imshow("img.png", *birdeye_img);
    cv::waitKey(0);





}






void detect_line(const cv::Mat undistort_img,
                 const RoiData &roi_data,
                 std::vector<cv::Point2f> &point_l,
                 std::vector<cv::Point2f> &point_r,
                 const int &canny_threshold)
{
  if (undistort_img.empty())
    printf("Image read error!\n");

  cv::Mat grad_x, img_canny;
  cv::Point2f point_tmp;
  cv::Mat img_median;
  cv::medianBlur(undistort_img, img_median, 7); // median filter
  cv::GaussianBlur(img_median, img_canny, cv::Size(3, 3), 0, 0,
                   cv::BORDER_DEFAULT);

  cv::Canny(img_canny, img_canny, 5, canny_threshold,
            3); // canny operator to detect edge

  int width_top = roi_data.width_top < roi_data.width_bottom
                      ? roi_data.width_top
                      : roi_data.width_bottom;
  int width_bottom = roi_data.width_top > roi_data.width_bottom
                         ? roi_data.width_top
                         : roi_data.width_bottom;
  int height_min = roi_data.height_min < roi_data.height_max
                       ? roi_data.height_min
                       : roi_data.height_max;
  int height_max = roi_data.height_min > roi_data.height_max
                       ? roi_data.height_min
                       : roi_data.height_max;
  width_top = width_top < 0 ? 0 : width_top;
  width_top = width_top > img_canny.cols ? img_canny.cols : width_top;
  width_bottom = width_bottom < 0 ? 0 : width_bottom;
  width_bottom = width_bottom > img_canny.cols ? img_canny.cols : width_bottom;
  height_min = height_min < 0 ? 0 : height_min;
  height_min = height_min > img_canny.rows ? img_canny.rows : height_min;
  height_max = height_max < 0 ? 0 : height_max;
  height_max = height_max > img_canny.rows ? img_canny.rows : height_max;

  cv::Mat roi = cv::Mat::zeros(img_canny.size(), CV_8U);
  cv::Mat img_canny_roi;
  std::vector<std::vector<cv::Point>> roi_contour;
  std::vector<cv::Point> roi_pts;
  roi_pts.push_back(cv::Point((img_canny.cols - width_top) / 2, height_min));
  roi_pts.push_back(cv::Point((img_canny.cols + width_top) / 2, height_min));
  roi_pts.push_back(cv::Point((img_canny.cols + width_bottom) / 2, height_max));
  roi_pts.push_back(cv::Point((img_canny.cols - width_bottom) / 2, height_max));
  roi_contour.push_back(roi_pts);
  cv::drawContours(roi, roi_contour, -1, cv::Scalar::all(255), -1);
  img_canny.copyTo(img_canny_roi, roi);

  cv::Mat img_show_edge;
  cv::cvtColor(img_median, img_show_edge, CV_GRAY2RGB);

  cv::Scharr(img_median, grad_x, CV_8UC1, 1, 0, 1, 0, cv::BORDER_DEFAULT);
  cv::Mat lane_edge =
      cv::Mat::zeros(img_canny_roi.rows, img_canny_roi.cols, CV_8UC1);
  const int horizontal_edge_thred = 30;
  //--------select marking lane edge--------//
  //--------using sobel gradient in x-axis with mask calculated above--------//
  int left_start_col = (img_canny.cols - width_bottom) / 2,
      left_end_col = img_canny_roi.cols / 2;
  int right_start_col = img_canny_roi.cols / 2 + 1,
      right_end_col = (img_canny.cols + width_bottom) / 2;
  // point_l : left img point after canny operator to fit lane.
  // point_r : same to above.
  for (int i = height_min; i < height_max; i++) {
    // left image
    for (int j_l = left_start_col; j_l < left_end_col; j_l++) {
      if (img_canny_roi.at<uchar>(i, j_l) < 200)
        continue;
      uchar grad_x_tmp = grad_x.at<uchar>(i, j_l);
      if (int(grad_x_tmp) < 100) {
        // discard horizontal line
        if (img_canny_roi.at<uchar>(i - 1, j_l - 1) < horizontal_edge_thred &&
            img_canny_roi.at<uchar>(i - 1, j_l) < horizontal_edge_thred &&
            img_canny_roi.at<uchar>(i - 1, j_l + 1) < horizontal_edge_thred &&
            img_canny_roi.at<uchar>(i + 1, j_l - 1) < horizontal_edge_thred &&
            img_canny_roi.at<uchar>(i + 1, j_l) < horizontal_edge_thred &&
            img_canny_roi.at<uchar>(i + 1, j_l + 1) < horizontal_edge_thred) {
          img_show_edge.at<cv::Vec3b>(i, j_l) = cv::Vec3b(0, 0, 255);
          continue;
        }

        lane_edge.at<uchar>(i, j_l) = 255;
        img_show_edge.at<cv::Vec3b>(i, j_l) = cv::Vec3b(0, 255, 0);
        point_tmp.x = j_l;
        point_tmp.y = i;
        point_l.push_back(point_tmp);
        // point_l.push_back(cv::Point(i, j_l));
      }
    }
    // right image
    for (int j_r = right_start_col; j_r < right_end_col; j_r++) {
      if (img_canny_roi.at<uchar>(i, j_r) < 200)
        continue;
      uchar grad_x_tmp = grad_x.at<uchar>(i, j_r);
      if (int(grad_x_tmp) > 100) {
        // discard horizontal line
        if (img_canny_roi.at<uchar>(i - 1, j_r - 1) < horizontal_edge_thred &&
            img_canny_roi.at<uchar>(i - 1, j_r) < horizontal_edge_thred &&
            img_canny_roi.at<uchar>(i - 1, j_r + 1) < horizontal_edge_thred &&
            img_canny_roi.at<uchar>(i + 1, j_r - 1) < horizontal_edge_thred &&
            img_canny_roi.at<uchar>(i + 1, j_r) < horizontal_edge_thred &&
            img_canny_roi.at<uchar>(i + 1, j_r + 1) < horizontal_edge_thred) {
          img_show_edge.at<cv::Vec3b>(i, j_r) = cv::Vec3b(0, 0, 255);
          continue;
        }

        lane_edge.at<uchar>(i, j_r) = 255;
        img_show_edge.at<cv::Vec3b>(i, j_r) = cv::Vec3b(0, 255, 0);
        point_tmp.x = j_r;
        point_tmp.y = i;
        point_r.push_back(point_tmp);
        // point_r.push_back(cv::Point(i, j_r));
      }
    }
  }
  cv::drawContours(img_show_edge, roi_contour, -1, cv::Scalar(255, 0, 0), 1);
  cv::imwrite("lane_edge.png", img_show_edge);
  cv::resize(img_show_edge, img_show_edge,
             cv::Size(img_show_edge.cols / 2, img_show_edge.rows / 2));
  cv::imshow("lane_edge.png", img_show_edge);

  cv::imshow("lane_edge", lane_edge);
  cv::waitKey(0);
}


//====================================================================//
//Program:RANSAC直线拟合
//RANSAC 拟合2D 直线
//输入参数：points--输入点集
//        iterations--迭代次数
//        sigma--数据和模型之间可接受的差值,车道线像素宽带一般为10左右
//              （Parameter use to compute the fitting score）
//        k_min/k_max--拟合的直线斜率的取值范围.
//                     考虑到左右车道线在图像中的斜率位于一定范围内，
//                      添加此参数，同时可以避免检测垂线和水平线
//输出参数:line--拟合的直线参数,It is a vector of 4 floats
//              (vx, vy, x0, y0) where (vx, vy) is a normalized
//              vector collinear to the line and (x0, y0) is some
//              point on the line.
//返回值：无
void fitLineRansac(const std::vector<cv::Point2f>& points,
                   cv::Vec4f &line,
                   int iterations,
                   double sigma,
                   double k_min,
                   double k_max)
{
    unsigned int n = points.size();

    if(n<2)
    {
        return;
    }

    cv::RNG rng;
    double bestScore = -1.;
    for(int k=0; k<iterations; k++)
    {
        int i1=0, i2=0;
        while(i1==i2)
        {
            i1 = rng(n);
            i2 = rng(n);
        }
        const cv::Point2f& p1 = points[i1];
        const cv::Point2f& p2 = points[i2];

        cv::Point2f dp = p2-p1;//直线的方向向量
        dp *= 1./norm(dp);
        double score = 0;

        if(dp.y/dp.x<=k_max && dp.y/dp.x>=k_min )
        {
            for(int i=0; i<n; i++)
            {
                cv::Point2f v = points[i]-p1;
                double d = v.y*dp.x - v.x*dp.y;//向量a与b叉乘/向量b的摸.||b||=1./norm(dp)
                //score += exp(-0.5*d*d/(sigma*sigma));//误差定义方式的一种
                if( fabs(d)<sigma )
                    score += 1;
            }
        }
        if(score > bestScore)
        {
            line = cv::Vec4f(dp.x, dp.y, p1.x, p1.y);
            bestScore = score;
        }
    }
}


void rotation_homography(CalibParams* camera, Mat* R)
{
    Vec3f theta = cv::Vec3f(angle_to_radian(camera->pitch), angle_to_radian(camera->yaw), angle_to_radian(camera->roll));
    * R = eulerAnglesToRotationMatrix(theta);
}

// void fitLineRansac_deprecated(const std::vector<cv::Point>& points,
//                   cv::Vec4f &line,
//                   int iterations,
//                   double sigma,
//                   double k_min,
//                   double k_max)
// {
//   int count_num = points.size();
//   Point2D32f pointdata[count_num];
//   for (size_t m = 0; m < count_num; m++) {
//     pointdata[m].x = points[m].y;
//     pointdata[m].y = points[m].x;
//   }
//   // parameters of ransac
//   // the number of choosen points every time.
//   // do not choose too big number.
//   // because ransac will calculate many times
//   int num_for_estimate = 3;
//   float success_probability = 0.999f;
//   float max_outliers_percentage = 0.9f;
//   // save lane information.
//   // line_information[3] : y-average
//   // line_information[2] : x-average
//   // line_information[1] : refer to least square to calcualte k
//   // line_information[0] : refer to least square to calcualte k
//   float line_infomation[4] = {0.0};

//   // left img line
//   RansacLane2d(pointdata, count_num, num_for_estimate, success_probability,
//                max_outliers_percentage, line_infomation, true);
//   cout << "line" << line_infomation[0] << "," << line_infomation[1] << endl;
//   cout << "line" << line_infomation[2] << "," << line_infomation[3] << endl;
//   line[0] = line_infomation[0];
//   line[1] = line_infomation[1];
//   line[2] = line_infomation[2];
//   line[3] = line_infomation[3];
// }