#ifndef BLOBDETECTION_H
#define BLOBDETECTION_H

// C++ standard libraries
#include <cmath>
#include <vector>

// OpenCV libarries
#include <opencv2/features2d.hpp>

#define FILE_NAME "image776.jpg"
#define OUTPUT_FILE "output_776.jpg"

#define INDEX 2
#define BLUR_INDEX 9

#define MIN_RADIUS 15.0f
#define MAX_RADIUS 60.0f
#define EXPONENT 2.0f

class BlobDetector {

public:
	void detect(cv::Mat& image, cv::Mat& output, std::vector<cv::KeyPoint>& keypoints);

private:
	static const bool FILTER_BY_COLOR = false;
	static const uchar DEFAULT_COLOR = 255;

	static const bool FILTER_BY_AREA = true;
	const float MIN_AREA = std::pow(MIN_RADIUS, EXPONENT) * (float) CV_PI;
	const float MAX_AREA = std::pow(MAX_RADIUS, EXPONENT) * (float) CV_PI;

	static const bool FILTER_BY_CIRCULARITY = false;
	const float MIN_CIRCULARITY = 0.9f;
	const float MAX_CIRCULARITY = 1.0f;

	static const bool FILTER_BY_CONVEXITY = false;
	const float MIN_CONVEXITY = 0.9f;
	const float MAX_CONVEXITY = 1.0f;

	static const bool FILTER_BY_INTERTIA = true;
	const float MIN_INERTIA_RATIO = 0.75f;
	const float MAX_INERTIA_RATIO = 1.0f;
};

#endif