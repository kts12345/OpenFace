﻿#ifndef __O2G_PROFILE_HPP__
#define __O2G_PROFILE_HPP__

#include <string>
#include <vector>
#include <chrono>
#include <cmath>

#include "boost/filesystem.hpp"

#include <opencv2/opencv.hpp>

#include "openface.hpp"

#include "o2g_util.hpp"
#include "o2g.h"

#ifndef	M_PI
#define	M_PI	3.14159265358979323846
#endif

namespace o2g {

struct FittingInfo
{
  // 메모리에 저장되었냐.
  bool is_saved;

  /// 위치 정보
  cv::Vec3d rotation;
  cv::Vec3d translation;
  cv::Vec3d face_feature_point;

  /// 사진
  cv::Mat image;

  /// 저장 시각
  std::chrono::system_clock::time_point saved_time;
  /// 몇 번째 사진을 저장한 것인지
  int frame_count;
};

class O2gProfile {
public:
  void clear() {
    _profile_name.clear();
    _root_path.clear();
    _fitting_info.clear();
    _image_cols = 0;
    _image_rows = 0;
    _fov = 0;
  }
 

  void init(std::string root_path, std::string profile_name,
    float camera_fov, int face_size_hint) {
    _root_path = root_path;
    _fitting_info.clear();
    _fitting_info.resize(s_image_count);
    _fov = camera_fov;
    _face_size_hint = face_size_hint;
    _frame_count = 0;

    _profile_name = profile_name;


    // TODO snow : 디버깅 코드 주석처리되어야 함.
    // std::string output_codec = "DIVX";
    // _video_writer = cv::VideoWriter(_root_path + "/b.avi",
    //                                 CV_FOURCC(output_codec[0], output_codec[1], output_codec[2], output_codec[3]),
    //                                 30, cv::Size(cols, rows), true);
  }
  bool is_init() {
    return !_profile_name.empty();
  }

  const std::string& profile_name() {
    return _profile_name;
  }

  // profile_update 가 호출함.
  void save_image(const openface::FittingInfo& info, cv::Mat image) {
    ++_frame_count;

    // 스무쓰한 영상을 얻기 위해 추가. by kts123. 
    // 별 문제 없다면 아래와 같은 패턴으로 저장될 것이라서
    // is_saved_all()을 호출하면 가운데 패턴만 얻어서 들쭉날쭉이 없을 것이다.
    // <<<<<<<<             (중앙에서 왼쪽으로 돌린 후)  <- 이때 저장한 것은 
    // >>>>>>>>>>>>>>>>     (방향 틀어서 오른쪽으로 돌린 후) <- 이때 덮어 쓴다.
    //        <<<<<<<<<     (방향 틀어서 왼쪽으로 돌린다)  <- 이때는 저정하지 말아야 한다.
    if (is_saved_all())
      return;

    // output the tracked video
    // TODO snow : 디버깅 코드 주석처리되어야 함.
    //if (_video_writer.isOpened())
    //{
    //  _video_writer << image;
    //}

    auto radian = info.rotate[1];
    if (radian < s_min_radian - 0.25*s_radian_distance ||
      radian > s_max_radian + 0.25*s_radian_distance)
      return;

    int index = (radian - s_min_radian) / s_radian_distance + 0.5f;
    // 이런 일은 생기지 않지만.. 한 번 더 남겨둔다.
    if (index < 0 || index >(_fitting_info.size() - 1)) {
      printf("something wrong, in index");
      return;
    }
    // TODO snow : 대충 정한 4초 어떻게 해야할 듯.
    // 중앙을 제외하고는
    // 너무 오래(4초 이상) 지난 것으로는 엎어치지 않는다.
    //auto cur = std::chrono::system_clock::now();
    //if (index != (_fitting_info.size() / 2) &&
    //    _fitting_info[index].is_saved == true &&
    //    std::chrono::duration_cast<std::chrono::seconds>(cur - _fitting_info[index].saved_time).count() > 4)
    //  return;

    //float cur_index_radian = s_min_radian + index * (s_radian_distance);
    //auto saved_radian = _fitting_info[index].rotation[1];
    //if (_fitting_info[index].is_saved == false) {// ||
   //     abs(saved_radian - cur_index_radian) > abs(radian - cur_index_radian)) {
      image.copyTo(_fitting_info[index].image);
      _fitting_info[index].is_saved = true;
      _fitting_info[index].rotation = info.rotate;
      _fitting_info[index].rotation[1] *= _yaw_scale;
      _fitting_info[index].translation = info.translation;
      _fitting_info[index].face_feature_point = info.face_feature_point;
      _fitting_info[index].saved_time = std::chrono::system_clock::now();
      _fitting_info[index].frame_count = _frame_count;
    //}
  }

  openface::CameraParameter& cp(int cols = -1, int rows = -1) {
    if (cols == -1 || rows == -1)
      return _cp;

    if (_image_cols != cols || _image_rows != rows) {
      _cp = camera_parameters(0, 0, 0, 0, cols, rows);
    }
    return _cp;
  }
  int frame_count() {
    return _frame_count;
  }

  bool is_saved_all() {
    for (int i = 0; i < s_image_count; ++i) {
      if (false == _fitting_info[i].is_saved)
        return false;
    }
    return true;
  }

  static cv::Vec3d obj2world(const cv::Vec3d& obj_point,
    const cv::Vec3d& rotation,
    const cv::Vec3d& translation) {

    double boxVerts[] = { 0, 0, 0 };

    cv::Mat_<double> box = cv::Mat(1, 3, CV_64F, boxVerts).clone();
    for (auto i = 0; i < 3; ++i) {
      auto col = box.col(i);
      col = col + (obj_point[i]);// +glasses_center[i]);
    }

    cv::Matx33d rot = Euler2RotationMatrix(
      cv::Vec3d(rotation[0], rotation[1], rotation[2]));

    cv::Mat_<double> rotBox;

    // Rotate the box
    rotBox = cv::Mat(rot) * box.t();
    rotBox = rotBox.t();

    // Move the bounding box to head position
    rotBox.col(0) = rotBox.col(0) + translation[0];
    rotBox.col(1) = rotBox.col(1) + translation[1];
    rotBox.col(2) = rotBox.col(2) + translation[2];

    cv::Vec3d ret;
    ret = rotBox.row(0);

    cv::Vec3d pos(ret[0], ret[1], ret[2]);
    return pos;
  }



  int save_to_temp_file() {
    std::string path = _root_path + "/8_temp/0_user_profile";
    return save_to_file(path);
  }

  int save_to_file() {
    std::string path = _root_path + "/0_user_profile";
    return save_to_file(path);
  }

  int save_to_file(const std::string& path) {
    if (false == boost::filesystem::exists(_root_path))
      return O2G_ERROR_INVALID_ROOT_PATH;

    if (false == boost::filesystem::exists(path))
      boost::filesystem::create_directory(path);
    std::string profile_path = path + "/" + _profile_name;

    // 이미 있으면 모두 지운다.
    if (true == boost::filesystem::exists(profile_path)) {
      boost::system::error_code ec;
      boost::filesystem::remove_all(profile_path, ec);
      if (ec) {
        printf("fail to delete directory");
        printf(profile_path.c_str());
        return O2G_ERROR_ALREADY_EXIST_PROFILE;
      }
    }

    // 이번 프로파일을 저장할 폴더를 만든다.
    boost::filesystem::create_directory(profile_path);

    std::string file_name;
    char buf[255];
    for (int i = 0; i < _fitting_info.size(); ++i) {
      if (_fitting_info[i].is_saved == true) {
        auto index = -1 * i + _fitting_info.size() - 1;
        sprintf(buf, "%02d.png", index);
        file_name = buf;
        if (false == cv::imwrite(profile_path + "/" + file_name, _fitting_info[i].image)) {
          // 디렉토리를 삭제한다.
          boost::system::error_code ec;
          boost::filesystem::remove_all(profile_path, ec);
          if (ec) {
            // 디렉토리 삭제에 실패함
            // 하지만 뭔가 더 할 수 있는 것은 없음.
            printf("fail to delete directory");
            printf(profile_path.c_str());
          }
          return O2G_ERROR_FAIL_TO_SAVE_IMAGE;
        }
      }
    }
    std::ofstream of;
    of.open(profile_path + "/config.txt", std::ios::out | std::ios::app);
    if (false == of.is_open()) {
      // 디렉토리를 삭제한다.
      boost::system::error_code ec;
      boost::filesystem::remove_all(profile_path, ec);
      if (ec) {
        // 디렉토리 삭제에 실패함
        // 하지만 뭔가 더 할 수 있는 것은 없음.
        printf("fail to delete directory");
        printf(profile_path.c_str());
      }
      return O2G_ERROR_FAIL_TO_SAVE_IMAGE;
    }

    of << "fov = " << fov() << std::endl;
    of << "face_size = " << _face_size_hint << std::endl;
    of << "face_count = " << s_image_count << std::endl;

    for (int i = _fitting_info.size() - 1; i >= 0; --i) {
      if (_fitting_info[i].is_saved == false)
        continue;
      auto index = -1 * i + _fitting_info.size() - 1;

      sprintf(buf, "r%02d = %.2f, %.2f, %.2f", index,
        _fitting_info[i].rotation[0] * 1.f,
        _fitting_info[i].rotation[1] * (-1.f),
        _fitting_info[i].rotation[2] * (-1.f));
      of << buf << std::endl;
    }
    for (int i = _fitting_info.size() - 1; i >= 0; --i) {
      if (_fitting_info[i].is_saved == false)
        continue;
      auto index = -1 * i + _fitting_info.size() - 1;

      auto& fi = _fitting_info[i];
      auto gt = obj2world(fi.face_feature_point, fi.rotation, fi.translation);

      sprintf(buf, "t%02d = %.2f, %.2f, %.2f", index,
        gt[0] / 10.0f,
        gt[1] * (-1.0f) / 10.0f,
        gt[2] * (-1.0f) / 10.0f + 20.0f);
      of << buf << std::endl;
    }

    of.close();
    return O2G_ERROR_SUCCESS;
  }

  float fov() { 
    return std::atan(_cp.cy / _cp.fy)*2.0f * 180 / M_PI;
  }

private:
  // 저장할 이미지 개수
  static const int s_image_count = 15;

  // 각도 : 최소, 최대 각도
  static constexpr float s_min_radian = -0.35;
  static constexpr float s_max_radian = 0.35;

  // 15개 이미지가 이미지 사이에 가지는 각도 거리
  static constexpr float s_radian_distance = (s_max_radian - s_min_radian) /
    (s_image_count - 1.0f);

  // module 의 root path
  std::string _root_path;

  // 이름
  std::string _profile_name;

  // image_count개 이미지의 피팅정보
  std::vector<FittingInfo> _fitting_info;

  // 회전값 스케일 튜닝
  double _yaw_scale = 1.05;

  // 카메라 파라미터
  openface::CameraParameter _cp;

  // 얼마나 줄여서 align 정보 찾을 지 
  int   _fitting_width = 256;
public:
  float calc_scale(int cols) { return (_fitting_width * 1.0f) / (cols * 1.0f); }
  float yaw_scale() { return _yaw_scale; }

private:
  int _image_cols;
  int _image_rows;
  float _fov;
  int _face_size_hint;
  int _frame_count;

  //cv::VideoWriter _video_writer;
};

}  // end namespace o2g

#endif  // __O2G_PROFILE_HPP__
