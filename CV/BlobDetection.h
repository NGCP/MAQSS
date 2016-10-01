#ifndef BLOBDETECTION_H
#define BLOBDETECTION_H

#include <opencv2/features2d.hpp>
#include <vector>

class BlobDetector {

public:
	void detect(cv::Mat& image, cv::Mat& output, std::vector<cv::KeyPoint>& keypoints);

private:
	static const bool FILTER_BY_COLOR = false;
	static const uchar DEFAULT_COLOR = 255;

	static const bool FILTER_BY_AREA = true;
		const float MIN_AREA = 2000.0f;
		const float MAX_AREA = 3000.0f;

	static const bool FILTER_BY_CIRCULARITY = true;
		const float MIN_CIRCULARITY = 0.9f;
		const float MAX_CIRCULARITY = 1.0f;
	
	static const bool FILTER_BY_CONVEXITY = true;
		const float MIN_CONVEXITY = 0.9f;
		const float MAX_CONVEXITY = 1.0f;

	static const bool FILTER_BY_INTERTIA = true;
		const float MIN_INERTIA_RATIO = 0.9f;
		const float MAX_INERTIA_RATIO = 1.0f;
};

#endif