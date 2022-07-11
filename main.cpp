#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include "calibration.hpp"

using namespace cv;
using namespace std;


//to do:
//load from config file
//from zyx to yxz?
//CARLA sim data
const double fisheye_fu=320;
const double fisheye_fv=320;
const double fisheye_cu=640;
const double fisheye_cv=360;
const double dis_coeff_0=0.08309221636708493;
const double dis_coeff_1=0.01112126630599195;
const double dis_coeff_2=-0.008587261043925865;
const double dis_coeff_3=0.0008542188930970716;
const double front_camera_x=3.1367766857147217;
const double front_camera_y=0;//-0.0018368728924542665;
const double front_camera_z=1.049095630645752;
const double front_camera_yaw=0;
const double front_camera_roll=0;
const double front_camera_pitch=-45;

const double left_camera_x=0;
const double left_camera_y=-1.1947870254516602;
const double left_camera_z=1.049095630645752;
const double left_camera_yaw=-90.0;
const double left_camera_roll=0;
const double left_camera_pitch=-45;

const double rear_camera_x=-3.1367766857147217;
const double rear_camera_y=0;//-0.0018368728924542665;
const double rear_camera_z=1.049095630645752;
const double rear_camera_yaw=180;
const double rear_camera_roll=0;
const double rear_camera_pitch=-45;

const double right_camera_x=0;
const double right_camera_y=1.1947870254516602;
const double right_camera_z=1.049095630645752;
const double right_camera_yaw=90;
const double right_camera_roll=0;
const double right_camera_pitch=-45;

const double birdeye_fu=1/0.01;//320;
const double birdeye_fv=1/0.01;//320;
const double birdeye_cu=avm_w/2;
const double birdeye_cv=avm_h/2;


/* transform all four fisheye images with four cameras' parameters
 *
 * cameras: a pointer to camera_set struct holding parameters of all four cameras
 * all_img: a pointer to all four fisheye images
 */
void birdeye_transform(Mat *all_img, camera_set* cameras, camera_set* camera_v, vector<one_frame_lines_set>* multi_frame_set, vector<vanishing_pts>* vanishing_pts_set);
/* simulate the initial pose changes of four cameras
 *
 * cameras: a pointer to camera_set struct holding parameters of all four cameras
 */
bool BlurPose(camera_set* cameras);

int main() {
    
    // read file
    string path = "/home/deliadong/Job/SVS/hank/tesla-move/svs/1/tesla-move.mp4";
    //fourinone_34997.jpg
    //"/home/deliadong/Job/SVS/alex/1/CamA_20211028_064436_2x2_4SVS.mp4";
    VideoCapture cap(path);
    Mat img, img_crop;
    
    // input parameters for the camera set
    camera_set* cameras = new camera_set;
    camera_set* cameras_v = new camera_set;
    
    // input parameters for each camera
    CalibParams* front = camera_new("front",
     fisheye_fu, fisheye_fv, fisheye_cu, fisheye_cv,
     dis_coeff_0, dis_coeff_1, dis_coeff_2, dis_coeff_3,
     front_camera_x, front_camera_y, front_camera_z
     );
    front->roll = front_camera_roll;
    front->pitch = front_camera_pitch;
    front->yaw = front_camera_yaw;

    CalibParams* right = camera_new("right",
     fisheye_fu, fisheye_fv, fisheye_cu, fisheye_cv,
     dis_coeff_0, dis_coeff_1, dis_coeff_2, dis_coeff_3,
     right_camera_x, right_camera_y, right_camera_z
    );
    right->roll = right_camera_roll;
    right->pitch = right_camera_pitch;
    right->yaw = right_camera_yaw;


    CalibParams* left = camera_new("left",
     fisheye_fu, fisheye_fv, fisheye_cu, fisheye_cv,
     dis_coeff_0, dis_coeff_1, dis_coeff_2, dis_coeff_3,
     left_camera_x, left_camera_y, left_camera_z
    );
    left->roll = left_camera_roll;
    left->pitch = left_camera_pitch;
    left->yaw = left_camera_yaw;
    
    CalibParams* rear = camera_new("rear",
     fisheye_fu, fisheye_fv, fisheye_cu, fisheye_cv,
     dis_coeff_0, dis_coeff_1, dis_coeff_2, dis_coeff_3,
     rear_camera_x, rear_camera_y, rear_camera_z
    );
    rear->roll = rear_camera_roll;
    rear->pitch = rear_camera_pitch;
    rear->yaw = rear_camera_yaw;

    CalibParams* front_v = camera_new("front", birdeye_fu, birdeye_fv, birdeye_cu, birdeye_cv, 0, 0, 0, 0, 0, 0, 0);
    CalibParams* rear_v = camera_new("rear", birdeye_fu, birdeye_fv, birdeye_cu, birdeye_cv, 0, 0, 0, 0, 0, 0, 0);
    CalibParams* left_v = camera_new("left", birdeye_fu, birdeye_fv, birdeye_cu, birdeye_cv, 0, 0, 0, 0, 0, 0, 0);
    CalibParams* right_v = camera_new("right", birdeye_fu, birdeye_fv, birdeye_cu, birdeye_cv, 0, 0, 0, 0, 0, 0, 0);
    
    cameras->front = front;
    cameras->rear = rear;
    cameras->left = left;
    cameras->right = right;
    print_calib_params(cameras->front);
    BlurPose(cameras);
    print_calib_params(cameras->front);

    cameras_v->front = front_v;
    cameras_v->rear = rear_v;
    cameras_v->left = left_v;
    cameras_v->right = right_v;
   
    //cap.set(CAP_PROP_POS_FRAMES, 230);
    int frame_num = 50;
    
    while (true) {
        vector<one_frame_lines_set> multi_frame_set;
        vector<vanishing_pts> vanishing_pts_set;
        for (int i = 0; i < frame_num; i ++) {
            cout << "i = " << i << endl;
            if(!cap.read(img)) {
                cout << "no video frame" << endl;
                break;
            }
            // image size is [2560 x 1440]
            Mat img_crop, img_resize;
            
            birdeye_transform(&img, cameras, cameras_v, &multi_frame_set, &vanishing_pts_set);

            // cv::imshow("img.png", img);
            // cv::waitKey(0);
        }
        // vanishing_pts final_vpts;
        // cluster_vpt_set(&vanishing_pts_set, &final_vpts);
        // bool result = calibration(frame_num, cameras_v, &multi_frame_set, &final_vpts);
        // if (result == true) {
        //     for (int i = 0; i < frame_num; i ++) {
        //         std::vector<lanemarks> lines_orig;
        //         lines_orig.insert(lines_orig.end(), multi_frame_set[i].pre_filtered_lines->front_lanes.begin(), multi_frame_set[i].pre_filtered_lines->front_lanes.end());
        //         lines_orig.insert(lines_orig.end(), multi_frame_set[i].pre_filtered_lines->rear_lanes.begin(), multi_frame_set[i].pre_filtered_lines->rear_lanes.end());
        //         lines_orig.insert(lines_orig.end(), multi_frame_set[i].pre_filtered_lines->left_lanes.begin(), multi_frame_set[i].pre_filtered_lines->left_lanes.end());
        //         lines_orig.insert(lines_orig.end(), multi_frame_set[i].pre_filtered_lines->right_lanes.begin(), multi_frame_set[i].pre_filtered_lines->right_lanes.end());
                
        //         Mat img = Mat::zeros(Size(352, 704),CV_8UC1);
        //         for (uint32_t i = 0; i < lines_orig.size(); i++) {
        //             cv::Vec4f line = lines_orig[i].rising_edge;
        //             cv::circle(img, cv::Point2f(line[0], line[1]), 1, cv::Scalar(255,255,255), 1);
        //             cv::circle(img, cv::Point2f(line[2], line[3]), 1, cv::Scalar(255,255,255), 1);
        //             cv::line(img, cv::Point2f(line[0], line[1]), cv::Point2f(line[2], line[3]), Scalar(255,255,255), 1);
        //             line = lines_orig[i].falling_edge;
        //             cv::circle(img, cv::Point2f(line[0], line[1]), 1, cv::Scalar(255,255,255), 1);
        //             cv::circle(img, cv::Point2f(line[2], line[3]), 1, cv::Scalar(255,255,255), 1);
        //             cv::line(img, cv::Point2f(line[0], line[1]), cv::Point2f(line[2], line[3]), Scalar(255,255,255), 1);
        //         }
        //         cv::imshow("image", img);
        //         cv::waitKey(20);
        //     }
        // }
        // cout << "one cycle" << endl;
    }
    
    return 0;
  }


void birdeye_transform(Mat* all_img, camera_set* cameras, camera_set* camera_v, vector<one_frame_lines_set>* multi_frame_set, vector<vanishing_pts>* vanishing_pts_set)
{
    Mat birdeye_img(Size(avm_w, avm_h), CV_8UC3); //numbers from avm_w and avm_h
    Mat img_crop;
    one_frame_lines_set* res = one_frame_set_new();
    vanishing_pts* v_pts = new vanishing_pts;
    Mat birdeye_front, birdeye_left, birdeye_rear, birdeye_right;
    for (int i = 0; i < 4; i++) {
        if (i == 0) {
            Rect roi(0, 0, 1280, 720);
            //resize((*all_img)(roi), img_crop, Size(1344, 968));//why resize here?
            img_crop = (*all_img)(roi);
            //oneview_extract_line(&img_crop, &birdeye_front, cameras->front, camera_v->front);
            oneview_extract_line(&img_crop, &birdeye_front, cameras->front, camera_v->front, res, v_pts);
        } else if (i == 1) {
            Rect roi(1280, 0, 1280, 720);
            //resize((*all_img)(roi), img_crop, Size(1344, 968));
            img_crop = (*all_img)(roi);
            oneview_extract_line(&img_crop, &birdeye_left, cameras->left, camera_v->left, res, v_pts);
        } else if (i == 2) {
            Rect roi(0, 720, 1280, 720);
            //resize((*all_img)(roi), img_crop, Size(1344, 968));
            img_crop = (*all_img)(roi);
            oneview_extract_line(&img_crop, &birdeye_rear, cameras->rear, camera_v->rear, res, v_pts);
        } else if (i == 3) {
            Rect roi(1280, 720, 1280, 720);
            //resize((*all_img)(roi), img_crop, Size(1344, 968));
            img_crop = (*all_img)(roi);
            oneview_extract_line(&img_crop, &birdeye_right, cameras->right, camera_v->right, res, v_pts);
        }
    }
// preview the stitch result
    birdeye_img = ground_stitch(birdeye_front, birdeye_left, birdeye_rear, birdeye_right, avm_h, avm_w);
    //cv::imwrite("birdeye_front.png", birdeye_front);
    //cv::imwrite("birdeye_left.png", birdeye_left);
    //cv::imwrite("birdeye_rear.png", birdeye_rear);
    //cv::imwrite("birdeye_right.png", birdeye_right);
    cv::imshow("birdeye_img.png", birdeye_img);
    cv::waitKey(0);
    multi_frame_set->push_back(*res);
    vanishing_pts_set->push_back(*v_pts);
    

//    std::vector<lanemarks> lines_orig;
//    lines_orig.insert(lines_orig.end(), res->orig_lines->front_lanes.begin(), res->orig_lines->front_lanes.end());
//    lines_orig.insert(lines_orig.end(), res->orig_lines->front_stop_lines.begin(), res->orig_lines->front_stop_lines.end());
//    lines_orig.insert(lines_orig.end(), res->orig_lines->rear_lanes.begin(), res->orig_lines->rear_lanes.end());
//    lines_orig.insert(lines_orig.end(), res->orig_lines->rear_stop_lines.begin(), res->orig_lines->rear_stop_lines.end());
//    lines_orig.insert(lines_orig.end(), res->orig_lines->left_lanes.begin(), res->orig_lines->left_lanes.end());
//    lines_orig.insert(lines_orig.end(), res->orig_lines->right_lanes.begin(), res->orig_lines->right_lanes.end());

//    Mat img_orig = birdeye_img.clone();
//    for (uint32_t i = 0; i < lines_orig.size(); i++) {
//        cv::Vec4f line = lines_orig[i].rising_edge;
//        cv::circle(img_orig, cv::Point2f(line[0], line[1]), 1, cv::Scalar(255, 0, 255), 1);
//        cv::circle(img_orig, cv::Point2f(line[2], line[3]), 1, cv::Scalar(255, 0, 255), 1);
//        cv::line(img_orig, cv::Point2f(line[0], line[1]), cv::Point2f(line[2], line[3]), Scalar(255, 0, 0), 1);
//        line = lines_orig[i].falling_edge;
//        cv::circle(img_orig, cv::Point2f(line[0], line[1]), 1, cv::Scalar(255, 0, 255), 1);
//        cv::circle(img_orig, cv::Point2f(line[2], line[3]), 1, cv::Scalar(255, 0, 255), 1);
//        cv::line(img_orig, cv::Point2f(line[0], line[1]), cv::Point2f(line[2], line[3]), Scalar(0, 255, 0), 1);
//    }
//    cv::imshow("original", birdeye_img);

//    std::vector<lanemarks> lines_pre_filtered;
//    lines_pre_filtered.insert(lines_pre_filtered.end(), res->pre_filtered_lines->front_lanes.begin(), res->pre_filtered_lines->front_lanes.end());
//    lines_pre_filtered.insert(lines_pre_filtered.end(), res->pre_filtered_lines->front_stop_lines.begin(), res->pre_filtered_lines->front_stop_lines.end());
//    lines_pre_filtered.insert(lines_pre_filtered.end(), res->pre_filtered_lines->rear_lanes.begin(), res->pre_filtered_lines->rear_lanes.end());
//    lines_pre_filtered.insert(lines_pre_filtered.end(), res->pre_filtered_lines->rear_stop_lines.begin(), res->pre_filtered_lines->rear_stop_lines.end());
//    lines_pre_filtered.insert(lines_pre_filtered.end(), res->pre_filtered_lines->left_lanes.begin(), res->pre_filtered_lines->left_lanes.end());
//    lines_pre_filtered.insert(lines_pre_filtered.end(), res->pre_filtered_lines->right_lanes.begin(), res->pre_filtered_lines->right_lanes.end());

//    Mat img_pre_filtered = birdeye_img.clone();
//    for (uint32_t i = 0; i < lines_pre_filtered.size(); i++) {
//        cv::Vec4f line = lines_pre_filtered[i].rising_edge;
//        cv::circle(img_pre_filtered, cv::Point2f(line[0], line[1]), 1, cv::Scalar(255, 0, 255), 1);
//        cv::circle(img_pre_filtered, cv::Point2f(line[2], line[3]), 1, cv::Scalar(255, 0, 255), 1);
//        cv::line(img_pre_filtered, cv::Point2f(line[0], line[1]), cv::Point2f(line[2], line[3]), Scalar(255, 0, 0), 1);
//        line = lines_pre_filtered[i].falling_edge;
//        cv::circle(img_pre_filtered, cv::Point2f(line[0], line[1]), 1, cv::Scalar(255, 0, 255), 1);
//        cv::circle(img_pre_filtered, cv::Point2f(line[2], line[3]), 1, cv::Scalar(255, 0, 255), 1);
//        cv::line(img_pre_filtered, cv::Point2f(line[0], line[1]), cv::Point2f(line[2], line[3]), Scalar(0, 255, 0), 1);
//    }
//    cv::imshow("pre_filtered", img_pre_filtered);
    

//    std::vector<lanemarks> lines_final;
//    lines_final.insert(lines_final.end(), res->filtered_lines->front_lanes.begin(), res->filtered_lines->front_lanes.end());
// //    lines_final.insert(lines_final.end(), res->filtered_lines->front_stop_lines.begin(), res->filtered_lines->front_stop_lines.end());
//    lines_final.insert(lines_final.end(), res->filtered_lines->rear_lanes.begin(), res->filtered_lines->rear_lanes.end());
// //    lines_final.insert(lines_final.end(), res->filtered_lines->rear_stop_lines.begin(), res->filtered_lines->rear_stop_lines.end());
// //    lines_final.insert(lines_final.end(), res->filtered_lines->left_lanes.begin(), res->filtered_lines->left_lanes.end());
// //    lines_final.insert(lines_final.end(), res->filtered_lines->right_lanes.begin(), res->filtered_lines->right_lanes.end());

//    Mat img_final = birdeye_img.clone();
//    for (uint32_t i = 0; i < lines_final.size(); i++) {
//        cv::Vec4f line = lines_final[i].rising_edge;
//        cv::circle(img_final, cv::Point2f(line[0], line[1]), 1, cv::Scalar(255, 0, 255), 1);
//        cv::circle(img_final, cv::Point2f(line[2], line[3]), 1, cv::Scalar(255, 0, 255), 1);
//        cv::line(img_final, cv::Point2f(line[0], line[1]), cv::Point2f(line[2], line[3]), Scalar(255, 0, 0), 1);
//        line = lines_final[i].falling_edge;
//        cv::circle(img_final, cv::Point2f(line[0], line[1]), 1, cv::Scalar(255, 0, 255), 1);
//        cv::circle(img_final, cv::Point2f(line[2], line[3]), 1, cv::Scalar(255, 0, 255), 1);
//        cv::line(img_final, cv::Point2f(line[0], line[1]), cv::Point2f(line[2], line[3]), Scalar(0, 255, 0), 1);
//    }
//    cv::imshow("line extractor res", img_final);

}

//modify the yaw and pitch, unit:angle
//to do:
//generate random number ranges from -5 to +5
bool BlurPose(camera_set* cameras)
{
    //front camera
    cameras->front->pitch += 3;
    cameras->front->yaw += 3;
    cameras->front->roll = 0;
}