
#define _CRT_SECURE_NO_WARNINGS

#include <string>
#include <vector>
#include <chrono>

#include "boost/filesystem.hpp"

#include <opencv2/opencv.hpp>

#include "openface/openface.hpp"

#include "o2g.h"

namespace {
cv::Matx33d Euler2RotationMatrix(const cv::Vec3d& eulerAngles)
{
  cv::Matx33d rotation_matrix;

  double s1 = sin(eulerAngles[0]);
  double s2 = sin(eulerAngles[1]);
  double s3 = sin(eulerAngles[2]);

  double c1 = cos(eulerAngles[0]);
  double c2 = cos(eulerAngles[1]);
  double c3 = cos(eulerAngles[2]);

  rotation_matrix(0,0) = c2 * c3;
  rotation_matrix(0,1) = -c2 *s3;
  rotation_matrix(0,2) = s2;
  rotation_matrix(1,0) = c1 * s3 + c3 * s1 * s2;
  rotation_matrix(1,1) = c1 * c3 - s1 * s2 * s3;
  rotation_matrix(1,2) = -c2 * s1;
  rotation_matrix(2,0) = s1 * s3 - c1 * c3 * s2;
  rotation_matrix(2,1) = c3 * s1 + c1 * s2 * s3;
  rotation_matrix(2,2) = c1 * c2;

  return rotation_matrix;
}

//===========================================================================
// Visualisation functions
//===========================================================================
void Project(cv::Mat_<double>& dest, const cv::Mat_<double>& mesh, double fx, double fy, double cx, double cy)
{
  dest = cv::Mat_<double>(mesh.rows,2, 0.0);

  int num_points = mesh.rows;

  double X, Y, Z;


  cv::Mat_<double>::const_iterator mData = mesh.begin();
  cv::Mat_<double>::iterator projected = dest.begin();

  for(int i = 0;i < num_points; i++)
  {
    // Get the points
    X = *(mData++);
    Y = *(mData++);
    Z = *(mData++);
      
    double x;
    double y;

    // if depth is 0 the projection is different
    if(Z != 0)
    {
      x = ((X * fx / Z) + cx);
      y = ((Y * fy / Z) + cy);
    }
    else
    {
      x = X;
      y = Y;
    }

    // Project and store in dest matrix
    (*projected++) = x;
    (*projected++) = y;
  }

}

enum class GlassesPart
{
  Back,
  Front
};

void DrawGlassesBox(const openface::FittingInfo& fitting_info,
                    cv::Scalar color,
                    int thickness,
                    const openface::CameraParameter& cp,
                    GlassesPart draw_part,
                    cv::Mat image)
{
  double fs = 1.0; // face_scale_factor
  // The size of the head is roughly 200mm x 200mm x 200mm
  //double glasses_size[] = { 140, 30, 100 }; // 가로, 세로, 깊이
  double glasses_size[] = { 140 * fs, 30 * fs, 100 * fs }; // 가로, 세로, 깊이
  double boxVerts[] = {
    // 사각형
    // 안경 다리 특화
    // 1. 안경 다리가 살짝 벌어지는 것을 표현하기 위해 120% 로 계산해준다.
    // 2. feature point 보다 안경다리쪽이 살짝 올라가는 것(귀가 약간더 위) 인 것을 표현하기 위해
    //    -3 을 해주었음

    // front
    -glasses_size[0] / 2,         glasses_size[1],       0,
     glasses_size[0] / 2,         glasses_size[1],       0,
     glasses_size[0] / 2,         0,                     0,
     -glasses_size[0] / 2,        0,                     0,

     // left leg
     -glasses_size[0] / 2,        glasses_size[1]/3,     0,
     -glasses_size[0] / 2 * 1.2,  glasses_size[1]/3,     glasses_size[2],

     // right leg
     glasses_size[0] / 2,         glasses_size[1]/3,     0,
     glasses_size[0] / 2 * 1.2,   glasses_size[1]/3,     glasses_size[2],

     // 앞쪽에 눈 사이를 세로로 가로지르는 직선을 위한 값
     0,                           0,                     0,
     0,                           glasses_size[1],       0,
  };

  std::vector<std::pair<int, int>> edges;
  edges.push_back(std::pair<int, int>(4, 5));
  edges.push_back(std::pair<int, int>(6, 7));

  edges.push_back(std::pair<int, int>(0, 1));
  edges.push_back(std::pair<int, int>(1, 2));
  edges.push_back(std::pair<int, int>(2, 3));
  edges.push_back(std::pair<int, int>(3, 0));
  edges.push_back(std::pair<int, int>(8, 9));

  double glasses_center[] = { 0, -10*fs, -5*fs };
  cv::Mat_<double> box = cv::Mat(10, 3, CV_64F, boxVerts).clone();
  for (auto i = 0; i < std::size(glasses_center); ++i)
  {
    auto col = box.col(i);
    col = col + (fitting_info.face_feature_point[i] + glasses_center[i]);
  }
  //std::cout << "face_feature_point: " << face_feature_point[0] << ", " << face_feature_point[1] << ", " << face_feature_point[2] << std::endl;

  cv::Matx33d rot = Euler2RotationMatrix(cv::Vec3d(fitting_info.rotate[0],
                                                   1.4f * fitting_info.rotate[1],
                                                   fitting_info.rotate[2]));
  cv::Mat_<double> rotBox;

  // Rotate the box
  rotBox = cv::Mat(rot) * box.t();
  rotBox = rotBox.t();

  // Move the bounding box to head position
  rotBox.col(0) = rotBox.col(0) + fitting_info.translation[0];
  rotBox.col(1) = rotBox.col(1) + fitting_info.translation[1];
  rotBox.col(2) = rotBox.col(2) + fitting_info.translation[2];

  // draw the lines
  cv::Mat_<double> rotBoxProj;
  Project(rotBoxProj, rotBox, cp.fx, cp.fy, cp.cx, cp.cy);

  // For subpixel accuracy drawing
  const int draw_multiplier = 1 << 4;
  const int draw_shiftbits = 4;

  cv::Rect image_rect(0, 0, image.cols * draw_multiplier, image.rows * draw_multiplier);

  for (size_t i = 0; i < edges.size(); ++i)
  {
    cv::Mat_<double> begin;
    cv::Mat_<double> end;

    rotBoxProj.row(edges[i].first).copyTo(begin);
    rotBoxProj.row(edges[i].second).copyTo(end);
    if (draw_part == GlassesPart::Front && i == 0 &&
        (begin.at<double>(0) < end.at<double>(0))) // 왼다리
      continue;
    if (draw_part == GlassesPart::Front && i == 1 &&
        (begin.at<double>(0) > end.at<double>(0))) // 오른다리
      continue;

    cv::Point p1(cvRound(begin.at<double>(0) * (double)draw_multiplier), cvRound(begin.at<double>(1) * (double)draw_multiplier));
    cv::Point p2(cvRound(end.at<double>(0) * (double)draw_multiplier), cvRound(end.at<double>(1) * (double)draw_multiplier));

    // Only draw the line if one of the points is inside the image
    if (p1.inside(image_rect) || p2.inside(image_rect))
    {
      cv::line(image, p1, p2, color, thickness, CV_AA, draw_shiftbits);
    }

  }

}


cv::Mat create_mask(const cv::Size &size, const cv::Mat_<double>& points) {
  cv::Mat mask = cv::Mat::zeros(size, CV_8U);
  mask.setTo(0);

  if (points.empty()) return mask;

  //cv::circle(mask, cv::Point(30,30), 20, cv::Scalar(0), 0);
  //std::cout << "2 rows:" << points.rows << ", cols:" << points.cols << std::endl;
  std::vector<cv::Point> vp;
  vp.reserve(points.rows);
  for (auto i = 0; i < points.rows; ++i) {
    vp.push_back(cv::Point(points.at<double>(i, 0), points.at<double>(i, 1)));
  }
  const cv::Point* p[1] = { &vp[0] };
  cv::fillPoly(mask, p, &points.rows, 1, cv::Scalar(255));
  // for debugging, 라인만 그려서 마스크 위치가 적절한지 확인하기 위한 코드
  //cv::polylines(mask, p, &points.rows, 1, true, cv::Scalar(255));
  return mask;
}


// Drawing landmarks on a face image
void Draw(cv::Mat img, const cv::Mat_<double>& shape2D)
{
  int n = shape2D.rows;
  for( int i = 0; i < n; ++i)
  {    
    const int draw_multiplier = 1 << 4;
    const int draw_shiftbits = 4;
    int x = shape2D.at<double>(i, 0);
    int y = shape2D.at<double>(i, 1);
     cv::Point featurePoint(cvRound(x * (double)draw_multiplier), cvRound(y * (double)draw_multiplier));

     // A rough heuristic for drawn point size
     int thickness = (int)std::ceil(3.0* ((double)img.cols) / 640.0);
     int thickness_2 = (int)std::ceil(1.0* ((double)img.cols) / 640.0);

     cv::circle(img, featurePoint, 1 * draw_multiplier, cv::Scalar(0, 0, 255), thickness, CV_AA, draw_shiftbits);
     cv::circle(img, featurePoint, 1 * draw_multiplier, cv::Scalar(255, 0, 0), thickness_2, CV_AA, draw_shiftbits);
  }

}

void make_image(cv::Mat captured_image,
                const openface::FittingInfo& info,
                const std::function<void(GlassesPart, cv::Mat)>& draw_glasses,
                cv::Mat out) {

  //cv::Mat mask_nose = create_mask(captured_image.size(), info.nose_feature_points);
  cv::Mat mask_face_outline = create_mask(captured_image.size(), info.face_outline_feature_points);
  captured_image.copyTo(out);
  draw_glasses(GlassesPart::Back, out);
  captured_image.copyTo(out, mask_face_outline);
  draw_glasses(GlassesPart::Front, out);
}


}  // end of namespace

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

  /// 저장 시각
  std::chrono::system_clock::time_point saved_time;
  /// 몇 번째 사진을 저장한 것인지
  int frame_count;
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
    // // TODO snow : 디버깅 코드 주석처리되어야 함.
    // if (_video_writer.isOpened())
    // {
    //   _video_writer << image;
    // }

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
  int _frame_count;

  cv::VideoWriter _video_writer;
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

  // 머리가 과도하게 돌아가 있으면 이런 사진은 제외시킨다.
  float roll = info.rotate[2];
  float pitch = info.rotate[0];
  if (abs(roll) > 0.5 || abs(pitch) > 0.4) {
    printf("roll: %.2f, pitch: %.2f", roll, pitch);
    return O2G_ERROR_SUCCESS;
  }
  // roate : roll 을 0 으로 맞춘다.
  float angle = roll * (180.0 / 3.14);
  cv::Point center(captured_image.cols / 2, captured_image.rows / 2);
  cv::Mat mat_rotation = cv::getRotationMatrix2D(center, angle, 1);
  cv::warpAffine(captured_image, captured_image, mat_rotation, captured_image.size());
  info.rotate[2] = 0;

  // // TODO snow : debugging 용 코드 추후 주석처리해야함
  // const int thickness = (int)std::ceil(2.0* ((double)captured_image.cols) / 640.0);

  // int count = 0, last_count = 0;
  // auto draw_glasses = [&info, &thickness](GlassesPart part, cv::Mat out) {
  //   DrawGlassesBox(info,
  //                  cv::Scalar((1 - info.detection_certainty)*255.0, 0, info.detection_certainty * 255),
  //                  thickness,
  //                  o2g::g_profile.cp(),
  //                  part,
  //                  out);
  // };

  // cv::Mat print_image;
  // captured_image.copyTo(print_image);

  // // 안경 그리기
  // make_image(captured_image, info, draw_glasses, print_image);
  // // feature points 그리기
  // Draw(print_image, info.feature_points_2d);
  // char buf[255];
  // std::sprintf(buf, "frame_count : %d", o2g::g_profile.frame_count());
  // cv::putText(print_image, buf, cv::Point(10, 30), CV_FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(255, 0, 0));
  // std::sprintf(buf, "rotate, pitch:%.2f, yaw:%.2f, roll:%.2f",
  //              info.rotate[0], info.rotate[1], info.rotate[2]);
  // cv::putText(print_image, buf, cv::Point(10, 50), CV_FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(255, 0, 0));
  // std::sprintf(buf, "translation, x:%.2f, y:%.2f, z:%.2f",
  //              info.translation[0], info.translation[1], info.translation[2]);
  // cv::putText(print_image, buf, cv::Point(10, 70), CV_FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(255, 0, 0));

  // // -- end TODO snow

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
