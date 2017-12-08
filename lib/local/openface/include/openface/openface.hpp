#ifndef __OPENFACE_OPENFACE_H__
#define __OPENFACE_OPENFACE_H__

#include "opencv2/core.hpp"

#ifdef _MSC_VER
#define O2FACE_CALLCONV __stdcall
#else
#define O2FACE_CALLCONV 
#endif

#ifdef __cplusplus
extern "C" {
#endif

namespace openface {

void O2FACE_CALLCONV set_logger(void* logger);

/// @brief face landmark model init
void* O2FACE_CALLCONV init_model(const std::string& model_path);

/// @brief face landmark model deinit
void O2FACE_CALLCONV deinit_model(void* model);

struct FittingInfo
{
  /// 얼굴에서 눈 사이 feature point 의 3D 좌표 (29번 feature point)
  cv::Vec3d face_feature_point;

  /// 얼굴 이동 거리
  cv::Vec3d translation;

  /// 얼굴 회전각
  cv::Vec3d rotate;

  /// 얼굴 크기 규모
  /// ex) 2: 실제 얼굴 크기가 200mm 이고 image 내의 얼굴이 100mm 인 경우
  double scale;

  /// 얼굴 feature point 를 찾았을 때 확신정도
  /// 이 값이 너무 낮으면 안경을 그리지 않는 편이 더 나을 것이다.
  double detection_certainty;

  /// 얼굴 윤곽 2D feature point 좌표
  cv::Mat_<double> face_outline_feature_points;

  /// 코 2D feature point 좌표
  cv::Mat_<double> nose_feature_points;

  /// 전체 (68개) 2D feature point 좌표
  cv::Mat_<double> feature_points_2d;
};

struct CameraParameter
{
  /// 초점거리(focal length): fx, fy
  float fx;
  /// 초점거리(focal length): fx, fy
  float fy;
  /// 주점(principal point): cx, cy
  float cx;
  /// 주점(principal point): cx, cy
  float cy;
};

struct DebuggingInfo
{
  /// land mark 를 이미지에 표기
  bool draw_landmarks;
};

/// @brief face landmark model deinit
///
/// @param[in] model face landmark detect model
/// @param[in] image face image
/// @param[in] face_size_mm 얼굴 크기, 광대 위치의 얼굴 가로 크기 단위 millimeter
/// @param[in] camera_parameter 카메라 정보
/// @param[in] debugging_info 디버깅을 위한 파라미터들
/// @param[out] 안경 fitting을 위한 정보
void O2FACE_CALLCONV glasses_fitting_info(
  void* model,
  cv::Mat& image,
  double face_size_mm,
  const CameraParameter& camera_parameter,
  const DebuggingInfo& debugging_info,
  FittingInfo& info);


}    // end namespace openface

#ifdef __cplusplus
}
#endif

#endif   // __OPENFACE_OPENFACE_H__
