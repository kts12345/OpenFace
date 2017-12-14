// TestOpenface.cpp : Defines the entry point for the console application.
//

#define _CRT_SECURE_NO_WARNINGS

#include <array>
#include <chrono>
#include <fstream>
#include <functional>
#include <iostream>
#include <sstream>
#include <vector>

// OpenCV includes
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio/videoio.hpp"  // Video write
#include "opencv2/videoio/videoio_c.h"  // Video write

#include "boost/filesystem.hpp"

// openface lib
#include "openface/openface.hpp"

#include "LandmarkDetectorParameters.h"

#ifdef _DEBUG
#pragma comment(lib, "openface-vc141-mt-gd.lib")
#pragma comment(lib, "LandmarkDetector-vc141-mt-gd.lib")
#pragma comment(lib, "dlib-vc141-mt-gd.lib")
#pragma comment(lib, "opencv_world310.lib")
#pragma comment(lib, "tbb_debug.lib")
#pragma comment(lib, "libboost_filesystem-vc140-mt-gd-1_60.lib")
#else
#pragma comment(lib, "openface-vc141-mt.lib")
#pragma comment(lib, "LandmarkDetector.lib")
#pragma comment(lib, "dlib.lib")
#pragma comment(lib, "opencv_world310.lib")
#pragma comment(lib, "tbb.lib")
#pragma comment(lib, "libboost_filesystem-vc140-mt-1_60.lib")
#endif


#define INFO_STREAM( stream ) \
std::cout << stream << std::endl

#define WARN_STREAM( stream ) \
std::cout << "Warning: " << stream << std::endl

#define ERROR_STREAM( stream ) \
std::cout << "Error: " << stream << std::endl


static void printErrorAndAbort( const std::string & error )
{
    std::cout << error << std::endl;
    abort();
}

#define FATAL_STREAM( stream ) \
printErrorAndAbort( std::string( "Fatal error: " ) + stream )


// Extracting the following command line arguments -f, -op, -of, -ov (and possible ordered repetitions)
void get_video_input_output_params(std::vector<std::string> &input_video_files,
                                 std::vector<std::string> &output_files,
                                   std::vector<std::string> &output_video_files,
                                 std::string& output_codec, 
                                 std::vector<std::string> &arguments)
{
  bool* valid = new bool[arguments.size()];

  for(size_t i = 0; i < arguments.size(); ++i)
  {
    valid[i] = true;
  }

    // By default use DIVX codec
  output_codec = "DIVX";

  std::string input_root = "";
  std::string output_root = "";

  std::string separator = std::string(1, boost::filesystem::path::preferred_separator);

  // First check if there is a root argument (so that videos and outputs could be defined more easilly)
  for(size_t i = 0; i < arguments.size(); ++i)
  {
    if (arguments[i].compare("-root") == 0)
    {
      input_root = arguments[i + 1] + separator;
      output_root = arguments[i + 1] + separator;

      // Add the / or \ to the directory
      i++;
    }
    if (arguments[i].compare("-inroot") == 0)
    {
      input_root = arguments[i + 1] + separator;
      i++;
    }
    if (arguments[i].compare("-outroot") == 0)
    {
      output_root = arguments[i + 1] + separator;
      i++;
    }
  }

  for(size_t i = 0; i < arguments.size(); ++i)
  {
    if (arguments[i].compare("-f") == 0) 
    {                    
      input_video_files.push_back(input_root + arguments[i + 1]);
      valid[i] = false; 
      valid[i+1] = false;      
      i++;
    }    
    else if (arguments[i].compare("-of") == 0)
    {
      output_files.push_back(output_root + arguments[i + 1]);
      //create_directory_from_file(output_root + arguments[i + 1]);
      valid[i] = false;
      valid[i+1] = false;
      i++;
    }
    else if (arguments[i].compare("-ov") == 0)
    {
      output_video_files.push_back(output_root + arguments[i + 1]);
      //create_directory_from_file(output_root + arguments[i + 1]);
      valid[i] = false;
      valid[i+1] = false;
      i++;
    }    
    else if (arguments[i].compare("-oc") == 0)
    {
      if(arguments[i + 1].length() == 4)
        output_codec = arguments[i + 1];
    }
  }

  for(int i=arguments.size()-1; i >= 0; --i)
  {
    if(!valid[i])
    {
      arguments.erase(arguments.begin()+i);
    }
  }

}
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



//enum class GlassesPart
//{
//  LeftLeg,
//  RightLeg,
//  Front,
//  FrontAndLeftLeg,
//  FrontAndRightLeg,
//  All,
//};
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

  char buf[255];
  std::sprintf(buf, "rotate:%.2f, %.2f, %.2f", info.rotate[0], info.rotate[1], info.rotate[2]);
  cv::putText(out, buf, cv::Point(10, 30), CV_FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(255, 0, 0));
}

std::vector<std::string> get_arguments(int argc, char **argv)
{

  std::vector<std::string> arguments;

  for(int i = 0; i < argc; ++i)
  {
    arguments.push_back(std::string(argv[i]));
  }
  return arguments;
}


cv::VideoCapture get_video_capture(const std::vector<std::string>& files, int device)
{ 
  // Do some grabbing
  cv::VideoCapture video_capture;
  if( files.size() > 0 )
  {
    std::string current_file = files[0];
    if (!boost::filesystem::exists(current_file))
    {
      FATAL_STREAM("File does not exist");
      return video_capture;
    }

    current_file = boost::filesystem::path(current_file).generic_string();

    INFO_STREAM( "Attempting to read from file: " << current_file );
    video_capture = cv::VideoCapture( current_file );
  }
  else
  {
    INFO_STREAM( "Attempting to capture from device: " << device );
    video_capture = cv::VideoCapture( device );

    // Read a first frame often empty in camera
    cv::Mat captured_image;
    video_capture >> captured_image;
  }

  return std::move(video_capture);
}

struct SavedInfo
{
  bool is_saved;
  cv::Vec3d rotation;
  cv::Vec3d translation;
};

void save_image(openface::FittingInfo info, cv::Mat image, float distance,
	float min_radian, float max_radian,
	std::vector<SavedInfo>& saved_info,
	std::vector<cv::Mat>& saved_images) {
  auto radian = info.rotate[1];
	if (radian < min_radian - distance || radian > max_radian + distance)
		return;

	int index = (radian - min_radian) / distance;
	if (index < 0 || index > 14) {
	  printf("aaaaaaaaaaaaaaaaa");
	  exit(0);
    }
  float cur_index_radian = min_radian + index * (distance);
  auto saved_radian = saved_info[index].rotation[1];
  if (abs(saved_radian - cur_index_radian) > abs(radian - cur_index_radian)) {
    image.copyTo(saved_images[index]);
    saved_info[index].is_saved = true;
    saved_info[index].rotation = info.rotate;
    saved_info[index].translation = info.translation;
  }
}

int profile_module_init(
  char* assert_root_path
);

int profile_module_final();

// Getting a head pose estimate from the currently detected landmarks, with appropriate correction due to orthographic camera issue
// This is because rotation estimate under orthographic assumption is only correct close to the centre of the image
// This method returns a corrected pose estimate with respect to world coordinates (Experimental)
int main(int argc, char **argv)
{
  std::vector<std::string> arguments = get_arguments(argc, argv);

  // 저장할 이미지 개수
  const int image_count = 15;
  // 각도 : 최소, 최대 각도
  const float min_radian = -0.5;
  const float max_radian = 0.5;
  // 15개 이미지가 이미지 사이에 가지는 각도 거리
  float radian_distance = (max_radian - min_radian) / (image_count - 1);
  // 저장할 이미지
  std::vector<cv::Mat> saved_images(image_count);
  // 15개 이미지의 피팅정보
  std::vector<SavedInfo> saved_info(image_count);
  for (int i = 0; i < saved_info.size(); ++i) {
    saved_info[i].is_saved = false;
  }

  // By default try webcam 0
  int device = 0;

  LandmarkDetector::FaceModelParameters det_parameters(arguments);

  // Get the input output file parameters

  // Some initial parameters that can be overriden from command line  
  std::vector<std::string> files, output_video_files, out_dummy;
  // Indicates that rotation should be with respect to world or camera coordinates
  string output_codec;
  get_video_input_output_params(files, out_dummy, output_video_files, output_codec, arguments);

  // 파일이 주어져있으면 파일에서, 없으면 webcam 에서 이미지를 가져오는 video capture 를 만듬
  // cv::VideoCapture video_capture = cv::VideoCapture( "E:\\dev\\est\\o2_openface\\videos\\0294_02_004_angelina_jolie.avi" );
  cv::VideoCapture video_capture = get_video_capture(files, device);
  if (!video_capture.isOpened()) {
    FATAL_STREAM("Failed to open video source");
    return 1;
  }
  else {
    INFO_STREAM("Device or file opened");
  }

  cv::Mat captured_image;
  video_capture >> captured_image;
  captured_image = captured_image.t();

  // saving the videos -> 테스트용 코드, 피쳐 찾은 정보를 추가해서 비디오를 저장함.
  cv::VideoWriter writerFace;
  if (!output_video_files.empty())
  {
    try
    {
      //writerFace = cv::VideoWriter(output_video_files[0], CV_FOURCC(output_codec[0], output_codec[1], output_codec[2], output_codec[3]), 30, captured_image.size(), true);
      writerFace = cv::VideoWriter("f:\\b.avi", CV_FOURCC(output_codec[0], output_codec[1], output_codec[2], output_codec[3]), 30, captured_image.size(), true);
    }
    catch(cv::Exception e)
    {
      WARN_STREAM( "Could not open VideoWriter, OUTPUT FILE WILL NOT BE WRITTEN. Currently using codec " << output_codec << ", try using an other one (-oc option)");
    }
  }



  const int thickness = (int)std::ceil(2.0* ((double)captured_image.cols) / 640.0);

  // 안경이 그려질 이미지
  cv::Mat print_image(captured_image.rows, captured_image.cols, captured_image.type());

  // openface 초기화
  void* model = openface::init_model(arguments);
  // 카메라 파라미터 초기화
  auto cp = camera_parameters(0, 0, 0, 0, captured_image.cols, captured_image.rows);

  INFO_STREAM( "Starting tracking");
  int count = 0, last_count = 0;
  openface::FittingInfo info, prev_info;
  auto draw_glasses = [&info, &thickness, &cp](GlassesPart part, cv::Mat out) {
    DrawGlassesBox(info,
                   cv::Scalar((1 - info.detection_certainty)*255.0, 0, info.detection_certainty * 255),
                   thickness,
                   cp,
                   part,
                   out);
  };

  // 정보를 남기기 이한 각종 변수들
  auto sum_distance = 0;
  double fps = 0, frame_cnt = 0;
  auto begin = std::chrono::system_clock::now();
  auto end = std::chrono::system_clock::now();
  while (!captured_image.empty()) {    

    if (count % 3 != 0) {
      count++;
      continue;
    }

    cv::flip(captured_image, captured_image, 1);

    auto prev = prev_info.feature_points_2d.rows;
    auto cur = info.feature_points_2d.rows;
    info.feature_points_2d.copyTo(prev_info.feature_points_2d);
    prev = prev_info.feature_points_2d.rows;
    cur = info.feature_points_2d.rows;

    // openface 에서 피팅 정보를 얻어옴
    openface::glasses_fitting_info(model, captured_image, 200, cp, { false }, info);

    // 디버깅용.
    auto distance = [&]() {
      float res = 0;
      auto prev = prev_info.feature_points_2d.rows;
      auto cur = info.feature_points_2d.rows;
      if (prev_info.feature_points_2d.rows != info.feature_points_2d.rows)
        return res;
      for (int i = 0; i < info.feature_points_2d.rows; ++i)
        res += std::sqrt(prev_info.feature_points_2d.at<double>(0) - info.feature_points_2d.at<double>(0));
      return res;
    };

    // 안경 그리기
    make_image(captured_image, info, draw_glasses, print_image);

    // feature points 그리기
    Draw(print_image, info.feature_points_2d);

    sum_distance += distance();

    auto now = std::chrono::system_clock::now();
    if (std::chrono::duration_cast<std::chrono::milliseconds>(now - begin).count() > 1000 &&
        count != last_count) {
      end = std::chrono::system_clock::now();
      sum_distance = 0;
      double duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();
      fps =  double(1000.0) / (duration_ms / (count - last_count));
      frame_cnt = count - last_count;
      last_count = count;
      begin = std::chrono::system_clock::now();
    }

    char buf[255];
    //std::sprintf(buf, "distance:%.2f", sum_distance/100.0);
    //cv::putText(print_image, buf, cv::Point(10, 50), CV_FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(255, 0, 0));
    std::sprintf(buf, "fps:%.2f", fps);
    cv::putText(print_image, buf, cv::Point(10, 70), CV_FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(255, 0, 0));
    std::sprintf(buf, "frame_cnt:%d", count);
    cv::putText(print_image, buf, cv::Point(10, 50), CV_FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(255, 0, 0));
    std::sprintf(buf, "certainty:%.2f", info.detection_certainty);
    cv::putText(print_image, buf, cv::Point(10, 90), CV_FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(255, 0, 0));

      
    // output the tracked video
    if (!output_video_files.empty())
    {
      writerFace << print_image;
    }

	  // 저장할 이미지를 saved_images에 저장함.
    save_image(info, print_image, radian_distance, min_radian, max_radian,
		           saved_info, saved_images);

    // 화면에 이미지를 보여줌
    cv::namedWindow("tracking_result", 1);
    cv::imshow("tracking_result", print_image);
  
    char character_press = cv::waitKey(1);
    // quit the application
    if (character_press=='q') {
      return(0);
    }

    // 다음 이미지를 얻어옴
    video_capture >> captured_image;
    captured_image = captured_image.t();

    ++count;
  }

  // 15 개 이미지 저장
  for (int i=0; i<saved_images.size(); ++i) {
    // 아직도 초기값이라면, 이미지가 없다는 뜻.
    if (saved_info[i].is_saved == true) {
      cv::imwrite(std::string("f:\\tmp\\") + std::to_string(i) + std::string(".png"), saved_images[i]);
	  }
  }

  openface::deinit_model(model);

  return 0;
}

