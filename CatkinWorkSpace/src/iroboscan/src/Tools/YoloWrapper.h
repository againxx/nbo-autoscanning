#ifndef YOLO_WRAPPER_H_
#define YOLO_WRAPPER_H_

#include <unordered_map>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include <Utils/Resolution.h>
#include "arapaho.hpp"

class YoloWrapper
{
	public:
		YoloWrapper();
		~YoloWrapper();

		void process(unsigned char * rgb, unsigned short * labels, bool byHand = false);

		// GET functions
		const std::vector<unsigned short> & getRecognizedPatchesByYolo() const { return recognizedPatchesByYolo; }
		double getProbability() const { return probability; }

	private:
		void recognizeUsingYolo(const cv::Mat & rgbImage);
		void recognizeByHand(const cv::Mat & rgbImage);

		void findLabelsInBox(const cv::Mat & labelImage);

		static void onMouseCallback(int event, int x, int y, int flags, void * userData);
		void onMouseCallback(int event, int x, int y);

		std::vector<unsigned short> recognizedPatchesByYolo;
		double probability;

		ArapahoV2 * arapaho = nullptr;

		bool beginDrawingBox = false;
		bool finishDrawingBox = false;
		cv::Point beginPoint;
		cv::Point endPoint;
 		double paraMinRecognizedProportion = 0.8;
		int paraMinPatchSize = 20;
};

#endif /* YOLO_WRAPPER_H_ */
