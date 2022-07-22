#ifndef calibration_hpp
#define calibration_hpp

#include "ceres/ceres.h"
#include "line_extractor.hpp"

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::NumericDiffCostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;

// bool front_back_calibration(double frame_num, CalibParams* camera, vector<one_frame_lines_set>* res, Point2f* vp, const double h0, const double hmax);

// void side_camera_calibration(double frame_num, CalibParams* camera, vector<one_frame_lines_set>* res, Point2f* vp,
//                              const double h0, const double hmax, const double center_axis);

//bool calibration(double frame_num, camera_set* cameras, vector<one_frame_lines_set>* res, vanishing_pts* v_pts);
bool calibration(double frame_num, camera_set* cameras,
    const std::vector<std::vector<LaneCoef>> &lane_coef,
    const std::vector<VanishPoint> &vanish_point,
    const std::vector<float> &pitch_raw,
    CameraRotationEuler &euler_angle);


void solve_one_frame_cpy(Problem* problem, double* v, vector<lanemarks>* lanes, const double h0, const double hmax);

bool solve_multi_frame_cpy(int frame_num, double* v, CalibParams* camera, vector<one_frame_lines_set>* multi_frame_set,
                           const double h0, const double hmax);

// void points_to_param(double* param, Vec4f points);

// double get_yaw(CalibParams* camera, Point2f* vanishing_pt);

// void get_all_lines_from_pairs(vector<lanemarks>* lanes, vector<Vec4f>* lines);

// double c_py(double vx, double vy, double* l1,
//             double* l2, const double h0, const double hmax);

// double c_rl(double pitch, double yaw, double roll, double* l1_l, double* l1_r, double* l2_l, double* l2_r);

// double get_lane_width_from_params(double* l1, double* l2);

// void solve_one_frame_crl(Problem* problem, vector<lanemarks>* lanes, const double pitch, const double yaw, double* roll);

// bool solve_multi_frame_crl(int frame_num, CalibParams* camera, vector<one_frame_lines_set>* multi_frame_set, double* roll);

// double get_lane_width(Vec4f rising_edge, Vec4f falling_edge);

// double get_midpoint(double h, double* l1, double* l2);

// double c_sp(double h, double pitch, double yaw, double roll, double* side_l1, double* side_l2, double* front_back_l1, double* front_back_l2);

// void solve_one_side_csp(Problem* problem, lanemarks* side_line, lanemarks* front_back_line, const double roll, const double yaw, double h, double* pitch, vector<double*>* line_params);

// void solve_one_frame_csp(CalibParams* camera, Problem* problem, one_frame_lines* lanes,
//                          const double roll, const double yaw, const double center_axis,
//                          double front_h, double rear_h, int* cnt, double* pitch, vector<double*>* line_params);

// void solve_multi_frame_csp(CalibParams* camera, vector<one_frame_lines_set>* res, const double center_axis,
//                            double front_h, double rear_h, double frame_num, double* pitch);

// bool correspondency_check(lanemarks* side_lane, lanemarks* front_back_lane, double threshold, double h);

// bool pre_lanemark_pairing_check(vector<lanemarks>* lanes, double center_axis);

// void update_lanes(vector<lanemarks>* lanes, Mat* H);


struct CostFatorPitch {
  CostFatorPitch(Eigen::Vector3d left_start, Eigen::Vector3d left_end,
                 Eigen::Vector3d right_start, Eigen::Vector3d right_end,
                 double height)
      : left_start_(left_start), left_end_(left_end), right_start_(right_start),
        right_end_(right_end), height_(height) {}
  bool operator()(const double *const pitch, double *residual) const {

    // calculate bearings intersetion with ground //
    cv::Mat Twc = cv::Mat::zeros(4, 4, CV_64F);
    Twc.at<double>(0, 3) = 0;
    Twc.at<double>(1, 3) = -height_;
    Twc.at<double>(2, 3) = 0;
    Twc.at<double>(3, 3) = 1.0;

    Eigen::Matrix3d Rx =
        (Eigen::AngleAxisd(pitch[0], Eigen::Vector3d::UnitX())).matrix();
    Eigen::Matrix3d Ry =
        (Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY())).matrix();
    Eigen::Matrix3d Rz =
        (Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ())).matrix();
    Eigen::Matrix3d R = Rx * Rz * Ry;

    cv::Mat Rcw = (cv::Mat_<double>(3, 3) << R(0, 0), R(0, 1), R(0, 2), R(1, 0),
                   R(1, 1), R(1, 2), R(2, 0), R(2, 1), R(2, 2));

    cv::Mat Rwc = Rcw.inv();

    Rwc.copyTo(Twc.rowRange(0, 3).colRange(0, 3));

    cv::Mat p0_start = (cv::Mat_<double>(3, 1) << left_start_(0),
                        left_start_(1), left_start_(2));
    double k0_start = (-Twc.at<double>(1, 3) / (Rwc.row(1).t().dot(p0_start)));
    cv::Mat pw0_start = Rwc * (k0_start * p0_start) + Twc.rowRange(0, 3).col(3);

    cv::Mat p0_end =
        (cv::Mat_<double>(3, 1) << left_end_(0), left_end_(1), left_end_(2));
    double k0_end = (-Twc.at<double>(1, 3) / (Rwc.row(1).t().dot(p0_end)));
    cv::Mat pw0_end = Rwc * (k0_end * p0_end) + Twc.rowRange(0, 3).col(3);

    cv::Mat p1_start = (cv::Mat_<double>(3, 1) << right_start_(0),
                        right_start_(1), right_start_(2));
    double k1_start = (-Twc.at<double>(1, 3) / (Rwc.row(1).t().dot(p1_start)));
    cv::Mat pw1_start = Rwc * (k1_start * p1_start) + Twc.rowRange(0, 3).col(3);

    cv::Mat p1_end =
        (cv::Mat_<double>(3, 1) << right_end_(0), right_end_(1), right_end_(2));
    double k1_end = (-Twc.at<double>(1, 3) / (Rwc.row(1).t().dot(p1_end)));
    cv::Mat pw1_end = Rwc * (k1_end * p1_end) + Twc.rowRange(0, 3).col(3);

    double z0_start = pw0_start.at<double>(2, 0);
    double x0_start = pw0_start.at<double>(0, 0);

    double z0_end = pw0_end.at<double>(2, 0);
    double x0_end = pw0_end.at<double>(0, 0);

    double k = (z0_end - z0_start) / (x0_end - x0_start);
    double b = z0_end - k * x0_end;
    double r1 = (13.4 - b) / k;
    double r2 = (4 - b) / k;

    double z1_start = pw1_start.at<double>(2, 0);
    double x1_start = pw1_start.at<double>(0, 0);

    double z1_end = pw1_end.at<double>(2, 0);
    double x1_end = pw1_end.at<double>(0, 0);

    double k_ = (z1_end - z1_start) / (x1_end - x1_start);
    double b_ = z1_end - k_ * x1_end;
    double r3 = (13.4 - b_) / k_;
    double r4 = (4 - b_) / k_;
    residual[0] = (r1 - r3) - (r2 - r4);
    return true;
  }

private:
  const Eigen::Vector3d left_start_;
  const Eigen::Vector3d left_end_;
  const Eigen::Vector3d right_start_;
  const Eigen::Vector3d right_end_;
  const double height_;
};

struct CostFatorPitchYaw {
  CostFatorPitchYaw(Eigen::Vector3d left_start, Eigen::Vector3d left_end,
                    Eigen::Vector3d right_start, Eigen::Vector3d right_end,
                    double height)
      : left_start_(left_start), left_end_(left_end), right_start_(right_start),
        right_end_(right_end), height_(height) {}
  bool operator()(const double *const pitch, const double *const yaw,
                  double *residual) const {

    cv::Mat Twc = cv::Mat::zeros(4, 4, CV_64F);
    Twc.at<double>(0, 3) = 0;
    Twc.at<double>(1, 3) = -height_;
    Twc.at<double>(2, 3) = 0;
    Twc.at<double>(3, 3) = 1.0;

    Eigen::Matrix3d Rx =
        (Eigen::AngleAxisd(pitch[0], Eigen::Vector3d::UnitX())).matrix();
    Eigen::Matrix3d Ry =
        (Eigen::AngleAxisd(-yaw[0], Eigen::Vector3d::UnitY())).matrix();
    Eigen::Matrix3d Rz =
        (Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ())).matrix();
    //         Eigen::Matrix3d R = Rz * Ry * Rx;
    Eigen::Matrix3d R = Rx * Rz * Ry;

    cv::Mat Rcw = (cv::Mat_<double>(3, 3) << R(0, 0), R(0, 1), R(0, 2), R(1, 0),
                   R(1, 1), R(1, 2), R(2, 0), R(2, 1), R(2, 2));
    cv::Mat Rwc = Rcw.inv();

    Rwc.copyTo(Twc.rowRange(0, 3).colRange(0, 3));

    cv::Mat p0_start = (cv::Mat_<double>(3, 1) << left_start_(0),
                        left_start_(1), left_start_(2));
    double k0_start = (-Twc.at<double>(1, 3) / (Rwc.row(1).t().dot(p0_start)));
    cv::Mat pw0_start = Rwc * (k0_start * p0_start) + Twc.rowRange(0, 3).col(3);

    cv::Mat p0_end =
        (cv::Mat_<double>(3, 1) << left_end_(0), left_end_(1), left_end_(2));
    double k0_end = (-Twc.at<double>(1, 3) / (Rwc.row(1).t().dot(p0_end)));
    cv::Mat pw0_end = Rwc * (k0_end * p0_end) + Twc.rowRange(0, 3).col(3);

    cv::Mat p1_start = (cv::Mat_<double>(3, 1) << right_start_(0),
                        right_start_(1), right_start_(2));
    double k1_start = (-Twc.at<double>(1, 3) / (Rwc.row(1).t().dot(p1_start)));
    cv::Mat pw1_start = Rwc * (k1_start * p1_start) + Twc.rowRange(0, 3).col(3);

    cv::Mat p1_end =
        (cv::Mat_<double>(3, 1) << right_end_(0), right_end_(1), right_end_(2));
    double k1_end = (-Twc.at<double>(1, 3) / (Rwc.row(1).t().dot(p1_end)));
    cv::Mat pw1_end = Rwc * (k1_end * p1_end) + Twc.rowRange(0, 3).col(3);

    double z0_start = pw0_start.at<double>(2, 0);
    double x0_start = pw0_start.at<double>(0, 0);

    double z0_end = pw0_end.at<double>(2, 0);
    double x0_end = pw0_end.at<double>(0, 0);

    double k = (z0_end - z0_start) / (x0_end - x0_start);
    double b = z0_end - k * x0_end;
    double r1 = (13.4 - b) / k;
    double r2 = (4 - b) / k;

    double z1_start = pw1_start.at<double>(2, 0);
    double x1_start = pw1_start.at<double>(0, 0);

    double z1_end = pw1_end.at<double>(2, 0);
    double x1_end = pw1_end.at<double>(0, 0);

    double k_ = (z1_end - z1_start) / (x1_end - x1_start);
    double b_ = z1_end - k_ * x1_end;
    double r3 = (13.4 - b_) / k_;
    double r4 = (4 - b_) / k_;
    residual[0] = fabs(r1 - r3) - fabs(r2 - r4) + fabs(r1 - r2);
    return true;
  }

private:
  const Eigen::Vector3d left_start_;
  const Eigen::Vector3d left_end_;
  const Eigen::Vector3d right_start_;
  const Eigen::Vector3d right_end_;
  const double height_;
};


void OptimizePitchYawSingle(
    const LaneVanishPoint &lane_data, const float &camera_height,
    const CameraIntrinsic &intrinsic,
    float &pitch_optimize);
void OptimizePitchYaw(const std::vector<LaneVanishPoint> &lane_data,
                      const float &camera_height,
                      const CameraIntrinsic &intrinsic,
                      float &pitch_optimize, float &yaw_optimize);
#endif /* calibration_hpp */
