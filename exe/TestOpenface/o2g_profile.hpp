#ifndef __O2G_PROFILE_HPP__
#define __O2G_PROFILE_HPP__

#include <string>
#include <vector>
#include <chrono>

#include "boost/filesystem.hpp"

#include <opencv2/opencv.hpp>

#include "openface/openface.hpp"

#include "o2g_util.hpp"
#include "o2g.h"


namespace o2g {

struct FittingInfo
{
  // 메모리에 저장되었냐.
  bool is_saved;

  /// 위치 정보
  cv::Vec3d rotation;
  cv::Vec3d translation;
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

  void init(std::string root_path, std::string profile_name, int cols, int rows,
            float camera_fov, int face_size_hint) {
    _root_path = root_path;
    _fitting_info.clear();
    _fitting_info.resize(s_image_count);
    _image_cols = cols;
    _image_rows = rows;
    _fov = camera_fov;
    _face_size_hint = face_size_hint;
    _frame_count = 0;

    // TODO snow : camera parameter 를 cx, cy, fx, fy 를
    //             아래 함수가 아닌 fov를 이용해서 만들도록 변경해야 함.
    _cp = camera_parameters(0, 0, 0, 0, cols, rows);

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
      
    // output the tracked video
    // TODO snow : 디버깅 코드 주석처리되어야 함.
    if (_video_writer.isOpened())
    {
      _video_writer << image;
    }

    auto radian = info.rotate[1];
  	if (radian < s_min_radian - s_radian_distance ||
        radian > s_max_radian + s_radian_distance)
  		return;

  	int index = (radian - s_min_radian) / s_radian_distance;
    // 이런 일은 생기지 않지만.. 한 번 더 남겨둔다.
  	if (index < 0 || index > (_fitting_info.size() - 1)) {
  	  printf("something wrong, in index");
      return;
    }
    // TODO snow : 대충 정한 4초 어떻게 해야할 듯.
    // 중앙을 제외하고는
    // 너무 오래(4초 이상) 지난 것으로는 엎어치지 않는다.
    auto cur = std::chrono::system_clock::now();
    if (index != (_fitting_info.size() / 2) &&
        _fitting_info[index].is_saved == true &&
        std::chrono::duration_cast<std::chrono::seconds>(cur - _fitting_info[index].saved_time).count() > 4)
      return;

    float cur_index_radian = s_min_radian + index * (s_radian_distance);
    auto saved_radian = _fitting_info[index].rotation[1];
    if (_fitting_info[index].is_saved == false ||
        abs(saved_radian - cur_index_radian) > abs(radian - cur_index_radian)) {
      image.copyTo(_fitting_info[index].image);
      _fitting_info[index].is_saved = true;
      _fitting_info[index].rotation = info.rotate;
      _fitting_info[index].translation = info.translation;
      _fitting_info[index].saved_time = std::chrono::system_clock::now();
      _fitting_info[index].frame_count = _frame_count;
    }
  }

  openface::CameraParameter& cp() {
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

  int save_to_file() {
    if (false == boost::filesystem::exists(_root_path))
      return O2G_ERROR_INVALID_ROOT_PATH;

    std::string path = _root_path + "\\profile";
    if (false == boost::filesystem::exists(path))
      boost::filesystem::create_directory(path);
    std::string profile_path = path + "\\" + _profile_name;
    if (true == boost::filesystem::exists(profile_path))
      return O2G_ERROR_ALREADY_EXIST_PROFILE;

    // 이번 프로파일을 저장할 폴더를 만든다.
    boost::filesystem::create_directory(profile_path);

    std::string file_name;
    char buf[255];
    for (int i = 0; i < _fitting_info.size(); ++i) {
      if (_fitting_info[i].is_saved == true) {
        sprintf(buf, "%02d.png", i);
        file_name = buf;
        if (false == cv::imwrite(profile_path + "\\" + file_name, _fitting_info[i].image)) {
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

    of << "fov = " << _fov << std::endl;
    of << "face_size = " << _face_size_hint << std::endl;
    for (int i = 0; i < _fitting_info.size(); ++i) {
      sprintf(buf, "r%02d = %.2f, %.2f, %.2f", i, _fitting_info[i].rotation[0],
              _fitting_info[i].rotation[1], _fitting_info[i].rotation[2]);
      of << buf << std::endl;
    }
    for (int i = 0; i < _fitting_info.size(); ++i) {
      sprintf(buf, "t%02d = %.2f, %.2f, %.2f", i, _fitting_info[i].translation[0],
              _fitting_info[i].translation[1], _fitting_info[i].translation[2]);
      of << buf << std::endl;
    }

    of.close();
    return O2G_ERROR_SUCCESS;
  }

private:
  // 저장할 이미지 개수
  static const int s_image_count = 15;

  // 각도 : 최소, 최대 각도
  static constexpr float s_min_radian = -0.5;
  static constexpr float s_max_radian = 0.5;

  // 15개 이미지가 이미지 사이에 가지는 각도 거리
  static constexpr float s_radian_distance = (s_max_radian - s_min_radian) /
                                             (s_image_count - 1.0f);

  // module 의 root path
  std::string _root_path;

  // 이름
  std::string _profile_name;

  // image_count개 이미지의 피팅정보
  std::vector<FittingInfo> _fitting_info;

  // 카메라 파라미터
  openface::CameraParameter _cp;

  int _image_cols;
  int _image_rows;
  float _fov;
  int _face_size_hint;
  int _frame_count;

  cv::VideoWriter _video_writer;
};

}  // end namespace o2g

#endif  // __O2G_PROFILE_HPP__
