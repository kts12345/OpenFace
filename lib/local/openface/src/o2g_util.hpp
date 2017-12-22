
#ifndef __O2G_UTIL_HPP__
#define __O2G_UTIL_HPP__

#define _CRT_SECURE_NO_WARNINGS

#include <string>
#include <vector>
#include <chrono>
#include <functional>

#include <opencv2/opencv.hpp>

#include "openface.hpp"

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

void draw_box(const openface::FittingInfo& info,
             const openface::CameraParameter& cp,
             cv::Mat image) {

  cv::Scalar color = cv::Scalar((1 - 0.9)*255.0, 0, 0.9 * 255);
  int thickness = 1;

	double boxVerts[] = {-1, 1, 2,
						1, 1, 2,
						1, 1, 0,
						-1, 1, 0,
						1, -1, 0,
						1, -1, 2,
						-1, -1, 2,
						-1, -1, 0};

	std::vector<std::pair<int,int>> edges;
	edges.push_back(std::pair<int,int>(0,1));
	edges.push_back(std::pair<int,int>(1,2));
	edges.push_back(std::pair<int,int>(2,3));
	edges.push_back(std::pair<int,int>(0,3));
	edges.push_back(std::pair<int,int>(2,4));
	edges.push_back(std::pair<int,int>(1,5));
	edges.push_back(std::pair<int,int>(0,6));
	edges.push_back(std::pair<int,int>(3,7));
	edges.push_back(std::pair<int,int>(6,5));
	edges.push_back(std::pair<int,int>(5,4));
	edges.push_back(std::pair<int,int>(4,7));
	edges.push_back(std::pair<int,int>(7,6));

	// The size of the head is roughly 200mm x 200mm x 200mm
	cv::Mat_<double> box = cv::Mat(8, 3, CV_64F, boxVerts).clone() * 100;

	cv::Matx33d rot = Euler2RotationMatrix(cv::Vec3d(info.rotate[0], info.rotate[1], info.rotate[2]));
	cv::Mat_<double> rotBox;
	
	// Rotate the box
	rotBox = cv::Mat(rot) * box.t();
	rotBox = rotBox.t();

	// Move the bounding box to head position
	rotBox.col(0) = rotBox.col(0) + info.translation[0] + info.face_feature_point[0];
	rotBox.col(1) = rotBox.col(1) + info.translation[1] + info.face_feature_point[1];
	rotBox.col(2) = rotBox.col(2) + info.translation[2] + info.face_feature_point[2];
	//rotBox.col(0) = rotBox.col(0) + info.face_feature_point[0];
	//rotBox.col(1) = rotBox.col(1) + info.face_feature_point[1];
	//rotBox.col(2) = rotBox.col(2) + info.face_feature_point[2];

	// draw the lines
	cv::Mat_<double> rotBoxProj;
	Project(rotBoxProj, rotBox, cp.fx, cp.fy, cp.cx, cp.cy);

  // For subpixel accuracy drawing
  const int draw_multiplier = 1 << 4;
  const int draw_shiftbits = 4;

	cv::Rect image_rect(0,0,image.cols * draw_multiplier, image.rows * draw_multiplier);
	
	for (size_t i = 0; i < edges.size(); ++i)
	{
		cv::Mat_<double> begin;
		cv::Mat_<double> end;
	
		rotBoxProj.row(edges[i].first).copyTo(begin);
		rotBoxProj.row(edges[i].second).copyTo(end);


		cv::Point p1(cvRound(begin.at<double>(0) * (double)draw_multiplier), cvRound(begin.at<double>(1) * (double)draw_multiplier));
		cv::Point p2(cvRound(end.at<double>(0) * (double)draw_multiplier), cvRound(end.at<double>(1) * (double)draw_multiplier));
		
		// Only draw the line if one of the points is inside the image
		if(p1.inside(image_rect) || p2.inside(image_rect))
		{
			cv::line(image, p1, p2, color, thickness, CV_AA, draw_shiftbits);
		}
		
	}

}

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

  for (auto i = 0; i < sizeof(glasses_center)/sizeof(glasses_center[0]); ++i)
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

openface::CameraParameter camera_parameters(float fx = 0, float fy = 0,
                                            float cx = 0, float cy = 0,
                                            int cols = 640, int rows = 480) {

  std::cout << "cols:" << cols << ", rows:" << rows;
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


}  // end of namespace


#endif  // __O2G_UTIL_HPP__
