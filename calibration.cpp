#include "calibration.hpp"

  static bool LaneCompare(const LaneVanishPoint &a, const LaneVanishPoint &b) {
    return a.vanish_point.y < b.vanish_point.y;
  }

//bool calibration(double frame_num, camera_set* cameras, vector<one_frame_lines_set>* res, vanishing_pts* v_pts)
bool calibration(double frame_num, camera_set* cameras,
    const std::vector<std::vector<LaneCoef>> &lane_coef,
    const std::vector<VanishPoint> &vanish_point,
    const std::vector<float> &pitch_raw,
    CameraRotationEuler &euler_angle)
{
    // each image has multi lane
    // each image has one vanish point
    cout<<"calibration:\n";
    cout<<"lane_coef[i].size():" << lane_coef.size() << endl;


    if (lane_coef.size() != vanish_point.size() ||
        lane_coef.size() != pitch_raw.size()) {
        std::cout << "lane_coef ||vanish_point ||pitch_raw size not equal. \n";
        return false;
    }

    // filter the lane coef
    std::vector<LaneVanishPoint> init_lane_data;
    std::vector<LaneVanishPoint> lane_datas;

    const int init_thredhold_count = 41;//81;
    float avg_vp_y = 0.0f;
    const int threadhold_vp_y = 5;
    for (size_t i = 0; i < lane_coef.size(); i++) {//(*res).size()
      // NOTE: only support 2 lane now
      int lane_num = lane_coef[i].size();//(*res)[i].orig_lines->front_lanes.size();
      if ( lane_num!= 2) {
        std::cout << "not found 2 lane in one image. \n";
        return false;
      }

      // TODO: can be better here
      if (i + 1 < init_thredhold_count) {
        LaneVanishPoint lane_vanish_point;
        lane_vanish_point.lane_coef = lane_coef[i];
        lane_vanish_point.vanish_point = vanish_point[i];
        lane_vanish_point.pitch_raw = pitch_raw[i];
        init_lane_data.push_back(lane_vanish_point);
        continue;
      } else if (i + 1 == init_thredhold_count) {
        sort(init_lane_data.begin(), init_lane_data.end(), LaneCompare);
        int i_begin = init_thredhold_count / 4;
        int i_end = init_thredhold_count * 3 / 4;
        for (int i = i_begin; i < i_end; ++i) {
          avg_vp_y += init_lane_data[i].vanish_point.y;
          lane_datas.push_back(init_lane_data[i]);
        }
        avg_vp_y /= (i_end - i_begin);
        init_lane_data.clear();
      }

      if (abs(vanish_point[i].y - avg_vp_y) < threadhold_vp_y) {
        LaneVanishPoint lane_vanish_point;
        lane_vanish_point.lane_coef = lane_coef[i];
        lane_vanish_point.vanish_point = vanish_point[i];
        lane_vanish_point.pitch_raw = pitch_raw[i];
        lane_datas.push_back(lane_vanish_point);
      }
    }

    cout << "filter the lane coef" << endl;
    if(lane_datas.size()==0) {
        cout << "no lane_datas for optimaize pitch yaw" << endl;
        return false;
    }

    cout << lane_datas.size() << " lane_datas for optimaize pitch yaw" << endl;

    CameraIntrinsic intrinsic = get_intrinsic(cameras->front);
    float pitch, yaw;
    float camera_height = 1.0;//to do
    OptimizePitchYaw(lane_datas, camera_height, intrinsic, pitch, yaw);

    // TODO: why R = Rx * Rz * Ry ?
    Eigen::Matrix3f Rx =
      (Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitX())).matrix();
    Eigen::Matrix3f Ry =
      (Eigen::AngleAxisf(-yaw, Eigen::Vector3f::UnitY())).matrix();
    Eigen::Matrix3f Rz =
      (Eigen::AngleAxisf(euler_angle.roll, Eigen::Vector3f::UnitZ())).matrix();
    // NOTE: roll is defind by input
    // Eigen::Matrix3f Rz =
    //     (Eigen::AngleAxisf(0.017, Eigen::Vector3f::UnitZ())).matrix();
    Eigen::Matrix3f R = Rx * Rz * Ry;

    cv::Mat Rcw = (cv::Mat_<float>(3, 3) << R(0, 0), R(0, 1), R(0, 2), R(1, 0),
                 R(1, 1), R(1, 2), R(2, 0), R(2, 1), R(2, 2));

    std::cout << "Rcw " << std::endl;
    std::cout << Rcw << std::endl;

    std::cout << " result " << std::endl;
    std::cout << " pitch " << pitch * 180 / CV_PI << std::endl;
    std::cout << " yaw " << yaw * 180 / CV_PI << std::endl;
    euler_angle.pitch = pitch;
    euler_angle.yaw = yaw;

    return true;
}

void solve_one_frame_cpy(Problem* problem, double* v, vector<lanemarks>* lanes, const double h0, const double hmax)
{
    
}


void CalBearings(const CameraIntrinsic &intrinsic,
                 const double k, const double b, Eigen::Vector3d &b1,
                 Eigen::Vector3d &b2) {
  {

    double x = (600 - b) / k;
    double y = 600;

    double cam_x = (x - intrinsic.cx) / intrinsic.fx;
    double cam_y = (y - intrinsic.cy) / intrinsic.fy;
    double cam_z = 1;
    double n = sqrt(cam_x * cam_x + cam_y * cam_y + cam_z * cam_z);
    b1 = Eigen::Vector3d(cam_x / n, cam_y / n, cam_z / n);
  }
  {
    double x = (960 - b) / k;
    double y = 960;
    double cam_x = (x - intrinsic.cx) / intrinsic.fx;
    double cam_y = (y - intrinsic.cy) / intrinsic.fy;
    double cam_z = 1;
    double n = sqrt(cam_x * cam_x + cam_y * cam_y + cam_z * cam_z);
    b2 = Eigen::Vector3d(cam_x / n, cam_y / n, cam_z / n);
  }
}

void OptimizePitchYawSingle(
    const LaneVanishPoint &lane_data, const float &camera_height,
    const CameraIntrinsic &intrinsic,
    float &pitch_optimize) {

  double pitch = double(pitch_optimize);
  double height = double(camera_height); // TODO: why not -camera_height
  Problem problem;
  double k1 = double(lane_data.lane_coef[0].coef_k);
  double b1 = double(lane_data.lane_coef[0].coef_b);
  double k2 = double(lane_data.lane_coef[1].coef_k);
  double b2 = double(lane_data.lane_coef[1].coef_b);
  Eigen::Vector3d bearing1;
  Eigen::Vector3d bearing2;
  Eigen::Vector3d bearing3;
  Eigen::Vector3d bearing4;
  CalBearings(intrinsic, k1, b1, bearing1, bearing2);
  CalBearings(intrinsic, k2, b2, bearing3, bearing4);

  CostFunction *cost_function =
      new NumericDiffCostFunction<CostFatorPitch, ceres::CENTRAL, 1, 1>(
          new CostFatorPitch(bearing1, bearing2, bearing3, bearing4, height));
  problem.AddResidualBlock(cost_function, NULL, &pitch);

  Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  options.logging_type = ceres::SILENT;
  options.minimizer_progress_to_stdout = false;
  Solver::Summary summary;
  Solve(options, &problem, &summary);
  std::cout << summary.BriefReport() << "\n";
  std::cout << " result  " << std::endl;
  std::cout << "  pitch " << pitch_optimize << " -> " << pitch << std::endl;
  pitch_optimize = float(pitch);
}


void OptimizePitchYaw(const std::vector<LaneVanishPoint> &lane_data,
                      const float &camera_height,
                      const CameraIntrinsic &intrinsic,
                      float &pitch_optimize, float &yaw_optimize) {

  double pitch_average = 0;
  for (size_t i = 0; i < lane_data.size(); i++) {
    LaneVanishPoint data = lane_data[i];
    pitch_average += double(data.pitch_raw);
  }
  pitch_average /= lane_data.size();
  std::cout << " Pitch initial value : " << pitch_average << std::endl;

  double pitch = pitch_average;
  double yaw = 0;
  double height = double(camera_height); // TODO: why not -camera_height

  Problem problem;
  for (int i = 0; i < int(lane_data.size()); ++i) {

    LaneVanishPoint data = lane_data[i];

    double k1 = double(data.lane_coef[0].coef_k);
    double b1 = double(data.lane_coef[0].coef_b);

    double k2 = double(data.lane_coef[1].coef_k);
    double b2 = double(data.lane_coef[1].coef_b);

    Eigen::Vector3d bearing1;
    Eigen::Vector3d bearing2;
    Eigen::Vector3d bearing3;
    Eigen::Vector3d bearing4;
    CalBearings(intrinsic, k1, b1, bearing1, bearing2);
    CalBearings(intrinsic, k2, b2, bearing3, bearing4);

    CostFunction *cost_function =
        new NumericDiffCostFunction<CostFatorPitchYaw, ceres::CENTRAL, 1, 1, 1>(
            new CostFatorPitchYaw(bearing1, bearing2, bearing3, bearing4,
                                  height));
    problem.AddResidualBlock(cost_function, NULL, &pitch, &yaw);
  }
  Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  options.logging_type = ceres::SILENT;
  options.minimizer_progress_to_stdout = false;
  Solver::Summary summary;
  Solve(options, &problem, &summary);
  std::cout << summary.BriefReport() << "\n";

  std::cout << " result  " << std::endl;
  std::cout << "  pitch " << pitch_average << " -> " << pitch << std::endl;
  std::cout << "  yaw " << 0.0 << " -> " << yaw << std::endl;

  pitch_optimize = float(pitch);
  yaw_optimize = float(yaw);
}