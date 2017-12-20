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
#include "o2g.h"

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
std::cout << "Info, "    << __FILE__ << ":" << __LINE__ << ", " << stream << std::endl

#define WARN_STREAM( stream ) \
std::cout << "Warning, " << __FILE__ << ":" << __LINE__ << ", " << stream << std::endl

#define ERROR_STREAM( stream ) \
std::cout << "Error, "   << __FILE__ << ":" << __LINE__ << ", " << stream << std::endl


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

  for (size_t i = 0; i < arguments.size(); ++i)
  {
    valid[i] = true;
  }

  // By default use DIVX codec
  output_codec = "DIVX";

  std::string input_root = "";
  std::string output_root = "";

  std::string separator = std::string(1, boost::filesystem::path::preferred_separator);

  // First check if there is a root argument (so that videos and outputs could be defined more easilly)
  for (size_t i = 0; i < arguments.size(); ++i)
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

  for (size_t i = 0; i < arguments.size(); ++i)
  {
    if (arguments[i].compare("-f") == 0)
    {
      input_video_files.push_back(input_root + arguments[i + 1]);
      valid[i] = false;
      valid[i + 1] = false;
      i++;
    }
    else if (arguments[i].compare("-of") == 0)
    {
      output_files.push_back(output_root + arguments[i + 1]);
      //create_directory_from_file(output_root + arguments[i + 1]);
      valid[i] = false;
      valid[i + 1] = false;
      i++;
    }
    else if (arguments[i].compare("-ov") == 0)
    {
      output_video_files.push_back(output_root + arguments[i + 1]);
      //create_directory_from_file(output_root + arguments[i + 1]);
      valid[i] = false;
      valid[i + 1] = false;
      i++;
    }
    else if (arguments[i].compare("-oc") == 0)
    {
      if (arguments[i + 1].length() == 4)
        output_codec = arguments[i + 1];
    }
  }

  for (int i = arguments.size() - 1; i >= 0; --i)
  {
    if (!valid[i])
    {
      arguments.erase(arguments.begin() + i);
    }
  }
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

// Getting a head pose estimate from the currently detected landmarks, with appropriate correction due to orthographic camera issue
// This is because rotation estimate under orthographic assumption is only correct close to the centre of the image
// This method returns a corrected pose estimate with respect to world coordinates (Experimental)
int main(int argc, char **argv)
{
  std::vector<std::string> arguments = get_arguments(argc, argv);

  // By default try webcam 0
  int device = 0;

  LandmarkDetector::FaceModelParameters det_parameters(arguments);

  std::string root_path("F:/dev/est/OpenFace/x64/Release");
  auto v = profile_module_init((char*)root_path.c_str());
  printf("======== v:%d", v);


  // Get the input output file parameters
  // Some initial parameters that can be overriden from command line  
  std::vector<std::string> files, output_video_files, out_dummy;
  // Indicates that rotation should be with respect to world or camera coordinates
  string output_codec;
  get_video_input_output_params(files, out_dummy, output_video_files, output_codec, arguments);

  // 파일이 주어져있으면 파일에서, 없으면 webcam 에서 이미지를 가져오는 video capture 를 만듬
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
  // 새로로 긴 이미지는 긴쪽이 가로가 되는 방향으로 돌아가는 문제가 있어서 임시로 이렇게 코딩함.
  captured_image = captured_image.t();

  // saving the videos -> 테스트용 코드, 피쳐 찾은 정보를 추가해서 비디오를 저장함.
  cv::VideoWriter writerFace;
  if (!output_video_files.empty())
  {
    try
    {
      //writerFace = cv::VideoWriter(output_video_files[0], CV_FOURCC(output_codec[0], output_codec[1], output_codec[2], output_codec[3]), 30, captured_image.size(), true);
      writerFace = cv::VideoWriter(".\\output\\b.avi", CV_FOURCC(output_codec[0], output_codec[1], output_codec[2], output_codec[3]), 30, captured_image.size(), true);
    }
    catch(cv::Exception e)
    {
      WARN_STREAM( "Could not open VideoWriter, OUTPUT FILE WILL NOT BE WRITTEN. Currently using codec " << output_codec << ", try using an other one (-oc option)");
    }
  }

  const int thickness = (int)std::ceil(2.0* ((double)captured_image.cols) / 640.0);

  INFO_STREAM( "Starting tracking");

  // 정보를 남기기 위한 각종 변수들
  double fps = 0, frame_cnt = 0;
  auto begin = std::chrono::system_clock::now();
  auto end = std::chrono::system_clock::now();
  int count = 0;

  std::string tmp_image = root_path + "\\a.png";
  std::string profile_name = "aa";
  v = profile_image_start((char*)profile_name.c_str(), 0, (char*)tmp_image.c_str(), 1);
  printf("======== v:%d", v);
  while (!captured_image.empty()) {    

    if (count % 3 != 0) {
      count++;
      continue;
    }

    cv::flip(captured_image, captured_image, 1);

    // tmp에 이미지 저장
    cv::imwrite(tmp_image, captured_image);

	  // 저장할 이미지를 saved_images에 저장함.
    // save_image(info, print_image, radian_distance, min_radian, max_radian,
		//            saved_info, saved_images);
    v = profile_image_update_file((char*)profile_name.c_str(),
                              (char*)tmp_image.c_str(), 720/2, 1280/2);
    printf("======== v:%d", v);


    // 화면에 이미지를 보여줌
    cv::namedWindow("tracking_result", 1);
    cv::imshow("tracking_result", captured_image);
  
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
  v = profile_image_final((char*)profile_name.c_str());
  printf("======== v:%d", v);
  v = profile_image_save((char*)profile_name.c_str());
  printf("======== v:%d", v);
  v = profile_module_final();
  printf("======== v:%d", v);
  return 0;
}

