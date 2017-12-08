
#include "openface/openface.hpp"

#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <vector>

#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"

#include <dlib/image_processing/frontal_face_detector.h>
//#include "dlib/algsd.h"

#include "LandmarkCoreIncludes.h"

  // for test code imshow
#include "opencv2/highgui/highgui.hpp"

namespace {

struct LandmarkDetectInfo
{
  LandmarkDetector::FaceModelParameters parameters;

  // The modules that are being used for tracking
  std::unique_ptr<LandmarkDetector::CLNF> model;
};

void get_face_landmark_3d(cv::Mat_<double>& out,
  const LandmarkDetector::CLNF& clnf_model,
  const std::vector<int>& landmark_ids) {
  out.create(landmark_ids.size(), 3);

  // 3D points
  cv::Mat_<double> landmarks_3D;
  clnf_model.pdm.CalcShape3D(landmarks_3D, clnf_model.params_local);

  landmarks_3D = landmarks_3D.reshape(1, 3);
  //std::cout << "dims:" << landmarks_3D.dims << ", rows:" << landmarks_3D.rows << ", cols:" << landmarks_3D.cols << std::endl;

  for (int i = 0; i < landmark_ids.size(); ++i) {
    out.at<double>(i, 0) = landmarks_3D.at<double>(0, landmark_ids[i]);
    out.at<double>(i, 1) = landmarks_3D.at<double>(1, landmark_ids[i]);
    out.at<double>(i, 2) = landmarks_3D.at<double>(2, landmark_ids[i]);
  }
}

cv::Mat_<double> get_landmark_2d_range(const LandmarkDetector::CLNF& clnf_model,
                                       const cv::Range& row_range) {
  cv::Mat landmarks = clnf_model.detected_landmarks.reshape(1, 2).t();
  return landmarks.rowRange(row_range);
}

cv::Mat_<double> get_landmark_2d(const LandmarkDetector::CLNF& clnf_model,
  const std::vector<int>& indices) {
  cv::Mat_<double> out;
  out.create(indices.size(), 2);
  cv::Mat landmarks = clnf_model.detected_landmarks.reshape(1, 2).t();
  for (auto i = 0; i < indices.size(); ++i) {
    out.at<double>(i, 0) = landmarks.at<double>(indices[i], 0);
    out.at<double>(i, 1) = landmarks.at<double>(indices[i], 1);
  }
  return out;
}

cv::Mat_<double> get_face_outline(const LandmarkDetector::CLNF& clnf_model) {

  cv::Mat_<double> ps = get_landmark_2d(clnf_model, { 0, 1, 2, 14, 15, 16 });

  // 얼굴 각도가 커지면, face line feature_point 를 잘 잡지 못하는 경향이 있어서,
  // 이에 대한 예외처리.
  // 눈썹 끝보다 얼굴 라인이 더 안쪽으로 나타나면 눈썹 끝 feature_point 에
  // x 좌표를 맞춰준다.
  cv::Mat_<double> ps_eyebrow = get_landmark_2d(clnf_model, { 17, 26 });
  if (ps_eyebrow.at<double>(0, 0) < ps.at<double>(0, 0)) {
    ps.at<double>(0, 0) = ps_eyebrow.at<double>(0, 0);
    ps.at<double>(1, 0) = ps_eyebrow.at<double>(0, 0);
    ps.at<double>(2, 0) = ps_eyebrow.at<double>(0, 0);
  }
  if (ps_eyebrow.at<double>(1, 0) > ps.at<double>(5, 0)) {
    ps.at<double>(3, 0) = ps_eyebrow.at<double>(1, 0);
    ps.at<double>(4, 0) = ps_eyebrow.at<double>(1, 0);
    ps.at<double>(5, 0) = ps_eyebrow.at<double>(1, 0);
  }
  cv::Mat_<double> p0 = ps.row(0);
  cv::Mat_<double> p2 = ps.row(2);
  cv::Mat_<double> p14 = ps.row(3);
  cv::Mat_<double> p16 = ps.row(5);
  cv::Mat_<double> p4 = p16 + (p16 - p14) * 2;
  cv::Mat_<double> p5 = p0 + (p0 - p2) * 2;

  ps.push_back(p4);
  ps.push_back(p5);
  ps.push_back(p0);
  return ps;
}

}

typedef void(*LoggerType)(const char*);
LoggerType g_logger_real = nullptr;

#define g_logger(X) \
{\
  if (g_logger_real) { \
    char buffer[256];\
    sprintf(buffer, "[line:%d]\t:%s", __LINE__, X);\
    g_logger_real(buffer);\
  } \
}

#ifdef __cplusplus
extern "C" {
#endif

namespace openface {
void O2FACE_CALLCONV set_logger(void* logger) {
  g_logger_real = (LoggerType)logger;
}


/// @brief face landmark model init
void* O2FACE_CALLCONV  init_model(const std::string& model_path) {

  std::vector<std::string> arguments;
  arguments.push_back(model_path);
  LandmarkDetector::FaceModelParameters parameters(arguments);

  // gaze 를 tracking 해야 더 잘찾는 것 같다.
	parameters.track_gaze = true;

  LandmarkDetectInfo* model = new LandmarkDetectInfo;
  model->parameters = parameters;
  model->model.reset(new LandmarkDetector::CLNF(parameters.model_location));
  return model;
}

/// @brief face landmark model deinit
void O2FACE_CALLCONV  deinit_model(void* model) {
  delete (LandmarkDetectInfo*)model;
}




  /// @brief face landmark model deinit
  ///
  /// @param[in] model face landmark detect model
  /// @param[in] image face image
  /// @param[in] face_size_mm 얼굴 크기, 광대 위치의 얼굴 가로 크기 단위 millimeter
  /// @param[in] camera_parameter 카메라 정보
  /// @param[out] 안경 fitting을 위한 정보
  void O2FACE_CALLCONV  glasses_fitting_info_imple(void* model,
    cv::Mat& image,
    double face_size_mm,
    const CameraParameter& camera_parameter,
    const DebuggingInfo& debugging_info,
    FittingInfo& info,
    bool is_video) {

  LandmarkDetectInfo* model2 = (LandmarkDetectInfo*)model;
  cv::Mat_<uchar> grayscale_image;
  if(image.channels() == 3) {
    cv::cvtColor(image, grayscale_image, CV_BGR2GRAY);
  }
  else {
    grayscale_image = image.clone();        
  }

  // The actual facial landmark detection / tracking
  //cv::Mat_<float> depth_image;
  bool detection_success = LandmarkDetector::DetectLandmarksInVideo(grayscale_image,
                                                                    //depth_image,
                                                                    *model2->model,
                                                                    model2->parameters);
  // Visualising the results
  // Drawing the facial landmarks on the face and the bounding box around it if tracking is successful and initialised
  double detection_certainty = model2->model->detection_certainty;

  auto pose = LandmarkDetector::GetPose(*(model2->model),
                                                      camera_parameter.fx, camera_parameter.fy,
                                                      camera_parameter.cx, camera_parameter.cy);

  if (debugging_info.draw_landmarks)
    LandmarkDetector::Draw(image, *model2->model);

  cv::Mat_<double> face_landmarks_3d;
  get_face_landmark_3d(face_landmarks_3d, *(model2->model), { 27 });  // center of eyes

  // set return value
  /// TODO snow : value가 copy 되게 하고 싶은데 깔끔하게 그렇게 되는 방법을 모르겠으니 일단 하나하나 복사한다.
  info.detection_certainty = detection_certainty;
  info.scale = 1;
  info.translation[0] = pose[0];
  info.translation[1] = pose[1];
  info.translation[2] = pose[2];
  info.rotate[0] = pose[3];
  info.rotate[1] = pose[4];
  info.rotate[2] = pose[5];
  auto feature_point = face_landmarks_3d.row(0);
  info.face_feature_point[0] = feature_point.at<double>(0);
  info.face_feature_point[1] = feature_point.at<double>(1);
  info.face_feature_point[2] = feature_point.at<double>(2);
  // face outline
  info.face_outline_feature_points = get_face_outline(*model2->model);
  // nose, 29, 32, 36 : 1부터 시작하는 Index
  info.nose_feature_points = get_landmark_2d(*model2->model, { 28, 31, 35, 28 });

  // 전체 2D feature point
  info.feature_points_2d = model2->model->detected_landmarks.reshape(1, 2).t();
}


  /// @brief face landmark model deinit
  ///
  /// @param[in] model face landmark detect model
  /// @param[in] image face image
  /// @param[in] face_size_mm 얼굴 크기, 광대 위치의 얼굴 가로 크기 단위 millimeter
  /// @param[in] camera_parameter 카메라 정보
  /// @param[out] 안경 fitting을 위한 정보
  void O2FACE_CALLCONV  glasses_fitting_info(void* model,
    cv::Mat& image,
    double face_size_mm,
    const CameraParameter& camera_parameter,
    const DebuggingInfo& debugging_info,
    FittingInfo& info) {
    const bool is_video_true = true;
    glasses_fitting_info_imple(model, 
      image, face_size_mm, camera_parameter, debugging_info, info, is_video_true);
  }

  /// @brief face landmark model deinit
  ///
  /// @param[in] model face landmark detect model
  /// @param[in] image face image
  /// @param[in] face_size_mm 얼굴 크기, 광대 위치의 얼굴 가로 크기 단위 millimeter
  /// @param[in] camera_parameter 카메라 정보
  /// @param[out] 안경 fitting을 위한 정보
  void O2FACE_CALLCONV  glasses_fitting_info_with_one_image(void* model,
    cv::Mat& image,
    double face_size_mm,
    const CameraParameter& camera_parameter,
    const DebuggingInfo& debugging_info,
    FittingInfo& info) {
    const bool is_video_false = false;
    glasses_fitting_info_imple(model,
      image, face_size_mm, camera_parameter, debugging_info, info, is_video_false);
  }


}    // end namespace openface


#ifdef __cplusplus
}
#endif

