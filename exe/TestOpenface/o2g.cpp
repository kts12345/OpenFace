﻿
#define _CRT_SECURE_NO_WARNINGS

#include <string>
#include <vector>
#include <chrono>

#include "boost/filesystem.hpp"

#include <opencv2/opencv.hpp>

#include "openface/openface.hpp"

#include "o2g_util.hpp"
#include "o2g_module.hpp"
#include "o2g_profile.hpp"
#include "o2g.h"


namespace {

o2g::O2gModule  g_module;
o2g::O2gProfile g_profile;

}  // end namespace

/// 피팅 라이브러리 모듈 초기화
int profile_module_init(
  char* root_path) {

  // module 초기화
  int result = g_module.init(root_path);
  if (result != O2G_ERROR_SUCCESS)
    return result;

  // profile 리셋
  g_profile.clear();

  return O2G_ERROR_SUCCESS;
}

/// 피팅 라이브러리 모듈 종료
int profile_module_final() {

  g_profile.clear();
  g_module.clear();

  return O2G_ERROR_SUCCESS;
}

int profile_image_start(
  char* profile_name,
  float camera_fov,
  char* image_temp_path,
  int face_size_hint) {

  if (false == g_module.is_init())
    return O2G_ERROR_MODULE_NO_INIT;
  if (true == g_profile.is_init())
    return O2G_ERROR_PROFILE_ALREADY_INIT;
  if (false == boost::filesystem::exists(image_temp_path))
    return O2G_ERROR_NO_IMAGE;

  cv::Mat captured_image = cv::imread(image_temp_path, -1);
  if (captured_image.data == NULL)
    return O2G_ERROR_READ_IMAGE;

  // profile 초기화
  g_profile.init(g_module.root_path(),
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

  if (false == g_module.is_init())
    return O2G_ERROR_MODULE_NO_INIT;
  if (false == g_profile.is_init())
    return O2G_ERROR_PROFILE_NO_INIT;
  if (false == boost::filesystem::exists(image_temp_path))
    return O2G_ERROR_NO_IMAGE;
  if (0 != g_profile.profile_name().compare(profile_name))
    return O2G_ERROR_WRONG_PROFILE_NAME;

  cv::Mat captured_image = cv::imread(image_temp_path, -1);
  if (captured_image.data == NULL)
    return O2G_ERROR_READ_IMAGE;

  openface::FittingInfo info;
  // openface 에서 피팅 정보를 얻어옴
  // TODO snow : 200 머리크기 하드코딩 되어있는 것 제거
  //             glasses_fitting_info 하위에서 현재 사용하지 않고 있음.
  openface::glasses_fitting_info(g_module.model(), captured_image,
                                 200, g_profile.cp(), { false }, info);

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

  // 회전시켰으므로 한번 더 구한다.
  openface::glasses_fitting_info(g_module.model(), captured_image,
                                 200, g_profile.cp(), { false }, info);

  // TODO snow : debugging 용 코드 추후 주석처리해야함
  const int thickness = (int)std::ceil(2.0* ((double)captured_image.cols) / 640.0);

  int count = 0, last_count = 0;
  auto draw_glasses = [&info, &thickness](GlassesPart part, cv::Mat out) {
    DrawGlassesBox(info,
                   cv::Scalar((1 - info.detection_certainty)*255.0, 0, info.detection_certainty * 255),
                   thickness,
                   g_profile.cp(),
                   part,
                   out);
  };

  cv::Mat print_image;
  captured_image.copyTo(print_image);

  // 안경 그리기
  make_image(captured_image, info, draw_glasses, print_image);
  // feature points 그리기
  Draw(print_image, info.feature_points_2d);
  // 네모 그리기
  draw_box(info, g_profile.cp(), print_image);
  char buf[255];
  std::sprintf(buf, "frame_count : %d", g_profile.frame_count());
  cv::putText(print_image, buf, cv::Point(10, 30), CV_FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(255, 0, 0));
  std::sprintf(buf, "rotate, pitch:%.2f, yaw:%.2f, roll:%.2f",
               info.rotate[0], info.rotate[1], info.rotate[2]);
  cv::putText(print_image, buf, cv::Point(10, 50), CV_FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(255, 0, 0));
  std::sprintf(buf, "translation, x:%.2f, y:%.2f, z:%.2f",
               info.translation[0], info.translation[1], info.translation[2]);
  cv::putText(print_image, buf, cv::Point(10, 70), CV_FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(255, 0, 0));

  // -- end TODO snow

  g_profile.save_image(info, print_image);

  return O2G_ERROR_SUCCESS;
}

/// 이미지 제공 끝났음.
int profile_image_final(
  char* profile_name) {
  if (false == g_module.is_init())
    return O2G_ERROR_MODULE_NO_INIT;
  if (false == g_profile.is_init())
    return O2G_ERROR_PROFILE_NO_INIT;
  if (false == g_profile.is_saved_all())
    return O2G_ERROR_FAIL_TO_MAKE_PROFILE;

  return O2G_ERROR_SUCCESS;
}

/// 프로필 이미지 결과 저장
int profile_image_save(
  char* profile_name) {
  if (false == g_module.is_init())
    return O2G_ERROR_MODULE_NO_INIT;
  if (false == g_profile.is_init())
    return O2G_ERROR_MODULE_NO_INIT;
  if (false == g_profile.is_saved_all())
    return O2G_ERROR_FAIL_TO_MAKE_PROFILE;

  auto r = g_profile.save_to_file();
  if (O2G_ERROR_SUCCESS != r)
    return r;
  g_profile.clear();
  return O2G_ERROR_SUCCESS;
}
