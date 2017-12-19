#include <string>
#include <vector>

#include "boost/filesystem.hpp"

#include <opencv2/opencv.hpp>

#include "openface/openface.hpp"

#include "o2g.h"


namespace o2g {

class O2gModule {
public:
  O2gModule() {
    _is_init = false;
    _model = nullptr;
  }
  void clear() {
    if (false == _is_init) return;

    _is_init = false;
    _root_path.clear();
    if (_model != nullptr) {
      openface::deinit_model(_model);
      _model = nullptr;
    }
  }
  int init(std::string root_path) {
    // 이미 초기화 되어 있음
    if (true == _is_init)
      return O2G_ERROR_SUCCESS;

    if (false == boost::filesystem::exists(root_path))
      return O2G_ERROR_INVALID_ROOT_PATH;

    std::string model_path = root_path;
    model_path += "/fitting_info/model/main_clm_general.txt";
    if (false == boost::filesystem::exists(model_path))
      return O2G_ERROR_NO_MODEL_INFO;

    std::vector<std::string> arguments;
    arguments.push_back(root_path);
    arguments.push_back("-mloc");
    arguments.push_back(model_path);
    arguments.push_back("-gaze");
    _model = openface::init_model(arguments);;

    _root_path = root_path;

    _is_init = true;

    return O2G_ERROR_SUCCESS;
  }
  bool is_init() {
    return _is_init;
  }
  const std::string& root_path() {
    return _root_path;
  }
  void* model() {
    return _model;
  }

private:
  bool _is_init;
  std::string _root_path;
  void* _model;

};

struct FittingInfo
{
  // 메모리에 저장되었냐.
  bool is_saved;

  /// 위치 정보
  cv::Vec3d rotation;
  cv::Vec3d translation;
  /// 사진
  cv::Mat image;
};

openface::CameraParameter camera_parameters(float fx = 0, float fy = 0,
                                            float cx = 0, float cy = 0,
                                            int cols = 640, int rows = 480) {
  bool cx_undefined = false;
  bool fx_undefined = false;
  if (cx == 0 || cy == 0)
  {
    cx_undefined = true;
  }
  if (fx == 0 || fy == 0)
  {
    fx_undefined = true;
  }

  if (cx_undefined)
  {
    cx = cols / 2.0f;
    cy = rows / 2.0f;
  }
  // Use a rough guess-timate of focal length
  if (fx_undefined)
  {
    fx = 500 * (cols / 640.0);
    fy = 500 * (rows / 480.0);

    fx = (fx + fy) / 2.0;
    fy = fx;
  }

  return openface::CameraParameter{ fx, fy, cx, cy };
}

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

    // TODO snow : camera parameter 를 cx, cy, fx, fy 를
    //             아래 함수가 아닌 fov를 이용해서 만들도록 변경해야 함.
    _cp = camera_parameters(0, 0, 0, 0, cols, rows);

    _profile_name = profile_name;
  }
  bool is_init() {
    return !_profile_name.empty();
  }

  const std::string& profile_name() {
    return _profile_name;
  }

  // profile_update 가 호출함.
  void save_image(const openface::FittingInfo& info, cv::Mat image) {
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
    float cur_index_radian = s_min_radian + index * (s_radian_distance);
    auto saved_radian = _fitting_info[index].rotation[1];
    if (_fitting_info[index].is_saved == false ||
        abs(saved_radian - cur_index_radian) > abs(radian - cur_index_radian)) {
      image.copyTo(_fitting_info[index].image);
      _fitting_info[index].is_saved = true;
      _fitting_info[index].rotation = info.rotate;
      _fitting_info[index].translation = info.translation;
    }
  }

  openface::CameraParameter& cp() {
    return _cp;
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
    for (int i = 0; i < _fitting_info.size(); ++i) {
      if (_fitting_info[i].is_saved == true) {
        file_name = std::to_string(i) + ".png";
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
    return O2G_ERROR_SUCCESS;
  }

private:
  // 저장할 이미지 개수
  static const int s_image_count = 15;

  // 각도 : 최소, 최대 각도
  static const float s_min_radian;
  static const float s_max_radian;

  // 15개 이미지가 이미지 사이에 가지는 각도 거리
  static const float s_radian_distance;

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
};

static O2gModule  g_module;
static O2gProfile g_profile;

const float O2gProfile::s_min_radian = -0.5;
const float O2gProfile::s_max_radian = 0.5;

// 이미지와 이미지 사이에 가지는 각도 거리
const float O2gProfile::s_radian_distance = (O2gProfile::s_max_radian - 
                                             O2gProfile::s_min_radian) / 
                                            (O2gProfile::s_image_count - 1);


}  // end namespace o2g

/// 피팅 라이브러리 모듈 초기화
int profile_module_init(
  char* root_path) {

  // module 초기화
  int result = o2g::g_module.init(root_path);
  if (result != O2G_ERROR_SUCCESS)
    return result;

  // profile 리셋
  o2g::g_profile.clear();

  return O2G_ERROR_SUCCESS;
}

/// 피팅 라이브러리 모듈 종료
int profile_module_final() {

  o2g::g_profile.clear();
  o2g::g_module.clear();

  return O2G_ERROR_SUCCESS;
}

int profile_image_start(
  char* profile_name,
  float camera_fov,
  char* image_temp_path,
  int face_size_hint) {

  if (false == o2g::g_module.is_init())
    return O2G_ERROR_MODULE_NO_INIT;
  if (true == o2g::g_profile.is_init())
    return O2G_ERROR_PROFILE_ALREADY_INIT;
  if (false == boost::filesystem::exists(image_temp_path))
    return O2G_ERROR_NO_IMAGE;

  cv::Mat captured_image = cv::imread(image_temp_path, -1);
  if (captured_image.data == NULL)
    return O2G_ERROR_READ_IMAGE;

  // profile 초기화
  o2g::g_profile.init(o2g::g_module.root_path(),
                      profile_name,
                      captured_image.cols, captured_image.rows,
                      camera_fov, face_size_hint);

  return O2G_ERROR_SUCCESS;
}

/// 이미지 제공
int profile_image_update_file(
  char* profile_name,
  char* image_temp_path,
  int face_outline_center_x,
  int face_outline_center_y) {

  if (false == o2g::g_module.is_init())
    return O2G_ERROR_MODULE_NO_INIT;
  if (false == o2g::g_profile.is_init())
    return O2G_ERROR_PROFILE_NO_INIT;
  if (false == boost::filesystem::exists(image_temp_path))
    return O2G_ERROR_NO_IMAGE;
  if (0 != o2g::g_profile.profile_name().compare(profile_name))
    return O2G_ERROR_WRONG_PROFILE_NAME;

  cv::Mat captured_image = cv::imread(image_temp_path, -1);
  if (captured_image.data == NULL)
    return O2G_ERROR_READ_IMAGE;

  openface::FittingInfo info;
  // openface 에서 피팅 정보를 얻어옴
  // TODO snow : 200 머리크기 하드코딩 되어있는 것 제거
  //             glasses_fitting_info 하위에서 현재 사용하지 않고 있음.
  openface::glasses_fitting_info(o2g::g_module.model(), captured_image,
                                 200, o2g::g_profile.cp(), { false }, info);
  o2g::g_profile.save_image(info, captured_image);

  return O2G_ERROR_SUCCESS;
}

/// 이미지 제공 끝났음.
int profile_image_final(
  char* profile_name) {
  if (false == o2g::g_module.is_init())
    return O2G_ERROR_MODULE_NO_INIT;
  if (false == o2g::g_profile.is_init())
    return O2G_ERROR_PROFILE_NO_INIT;
  if (false == o2g::g_profile.is_saved_all())
    return O2G_ERROR_FAIL_TO_MAKE_PROFILE;

  return O2G_ERROR_SUCCESS;
}

/// 프로필 이미지 결과 저장
int profile_image_save(
  char* profile_name) {
  if (false == o2g::g_module.is_init())
    return O2G_ERROR_MODULE_NO_INIT;
  if (false == o2g::g_profile.is_init())
    return O2G_ERROR_MODULE_NO_INIT;
  if (false == o2g::g_profile.is_saved_all())
    return O2G_ERROR_FAIL_TO_MAKE_PROFILE;

  auto r = o2g::g_profile.save_to_file();
  if (O2G_ERROR_SUCCESS != r)
    return r;
  o2g::g_profile.clear();
  return O2G_ERROR_SUCCESS;
}
