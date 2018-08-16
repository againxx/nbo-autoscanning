#include "YoloWrapper.h"

YoloWrapper::YoloWrapper()
 : arapaho(new ArapahoV2())
{
	std::string homePath = getenv("HOME");

	std::string tempStr = homePath + "/ProjectData/ArapahoConfig/coco.data";
	char * inputDataFile = new char[tempStr.size() + 1];
	tempStr.copy(inputDataFile, tempStr.size(), 0);
	inputDataFile[tempStr.size()] = '\0';

	tempStr = homePath + "/ProjectData/ArapahoConfig/yolo.cfg";
	char * inputCfgFile = new char[tempStr.size() + 1];
	tempStr.copy(inputCfgFile, tempStr.size(), 0);
	inputCfgFile[tempStr.size()] = '\0';

	tempStr = homePath + "/ProjectData/ArapahoConfig/yolo.weights";
	char * inputWeightsFile = new char[tempStr.size() + 1];
	tempStr.copy(inputWeightsFile, tempStr.size(), 0);
	inputWeightsFile[tempStr.size()] = '\0';

	ArapahoV2Params params;
	params.datacfg = inputDataFile;
	params.cfgfile = inputCfgFile;
	params.weightfile = inputWeightsFile;
	params.nms = 0.4;
	params.maxClasses = 2;

	int expectedW = 0, expectedH = 0;

	if (!arapaho->Setup(params, expectedW, expectedH))
	{
		std::cerr << "Error: arapaho setup failed!" << std::endl;
		if (arapaho)
			delete arapaho;
		arapaho = nullptr;
	}

	delete [] inputDataFile;
	delete [] inputCfgFile;
	delete [] inputWeightsFile;
}

YoloWrapper::~YoloWrapper()
{
	if (arapaho)
		delete arapaho;
}

void YoloWrapper::process(unsigned char * rgb, unsigned short * labels, bool byHand)
{
	cv::Mat rgbImage(Resolution::getInstance().height(), Resolution::getInstance().width(), CV_8UC3, rgb);
	cv::Mat labelImage(Resolution::getInstance().height(), Resolution::getInstance().width(), CV_16UC1, labels);
	cv::cvtColor(rgbImage, rgbImage, cv::COLOR_RGB2BGR);

	unsigned short maxLabel = 0;
	for (int i = 0; i < labelImage.rows; ++i)
	{
		for (int j = 0; j < labelImage.cols; ++j)
		{
			maxLabel = std::max(maxLabel, labelImage.at<ushort>(i, j));
		}
	}
	std::cout << "Label image max label: " << maxLabel << std::endl;

	/* TODO: use polymorphic to get rid of if-else <21-12-17, xiaxi> */
	if (byHand)
		recognizeByHand(rgbImage);
	else
		recognizeUsingYolo(rgbImage);

	findLabelsInBox(labelImage);

	delete [] labels;
}

void YoloWrapper::recognizeUsingYolo(const cv::Mat & rgbImage)
{
	recognizedPatchesByYolo.clear();
	probability = 0.5;
	beginPoint = cv::Point(0, 0);
	endPoint = cv::Point(0, 0);

	if (rgbImage.empty())
	{
		std::cerr << "Error: image is empty!" << std::endl;
		return;
	}

	std::cout << "Image width = " << rgbImage.size().width << ", height = " << rgbImage.size().height << std::endl;
	int objectNum = 0;
	arapaho->Detect(rgbImage, 0.24, 0.5, objectNum);

	box * boxes = nullptr;
	std::string * labels = nullptr;
	float * probabilities = nullptr;
	if (objectNum > 0)
	{
		boxes = new box[objectNum];
		labels = new std::string[objectNum];
		probabilities = new float[objectNum];
		arapaho->GetBoxes(boxes, labels, probabilities, objectNum);

		auto maxProbability = std::max_element(probabilities, probabilities + objectNum);
		probability = *maxProbability;
		int bestIndex = maxProbability - probabilities;

		beginPoint.x = 1 + rgbImage.size().width * (boxes[bestIndex].x - boxes[bestIndex].w / 2);
		beginPoint.y = 1 + rgbImage.size().height * (boxes[bestIndex].y - boxes[bestIndex].h / 2);
		endPoint.x = 1 + rgbImage.size().width * (boxes[bestIndex].x + boxes[bestIndex].w / 2);
		endPoint.y = 1 + rgbImage.size().height * (boxes[bestIndex].y + boxes[bestIndex].h / 2);

	}
	else
	{
		std::cout << "No object is recognized by YOLO!" << std::endl;
	}

	cv::rectangle(rgbImage, beginPoint, endPoint, cv::Scalar(0, 255, 0), 3);
	std::string windowName("Recognize object by YOLO");

	cv::startWindowThread();
	cv::namedWindow(windowName);
	cv::moveWindow(windowName, 800, 300);
	cv::imshow(windowName, rgbImage);
	cv::waitKey(0);
	cv::destroyWindow(windowName);

	if (boxes)
		delete [] boxes;
	if (labels)
		delete [] labels;
	if (probabilities)
		delete [] probabilities;
}

void YoloWrapper::recognizeByHand(const cv::Mat & rgbImage)
{
	recognizedPatchesByYolo.clear();
	beginDrawingBox = false;
	finishDrawingBox = false;

	std::string windowName("Recognize object by hand");

	cv::startWindowThread();
	cv::Mat tempImage;
	cv::namedWindow(windowName);
	cv::moveWindow(windowName, 800, 300);
	cv::setMouseCallback(windowName, onMouseCallback, this);

	while (1)
	{
		rgbImage.copyTo(tempImage);
		if (beginDrawingBox || finishDrawingBox)
			cv::rectangle(tempImage, beginPoint, endPoint, cv::Scalar(0, 255, 0), 3);

		cv::imshow(windowName, tempImage);
		int inputKey = cv::waitKey(10);
		if (inputKey == 13 || inputKey == 10)
		{
			cv::destroyWindow(windowName);
			break;
		}
	}
	probability = 1.0;
}

void YoloWrapper::findLabelsInBox(const cv::Mat & labelImage)
{
	std::unordered_map<unsigned short, int> totalLabelsCount;
	std::unordered_map<unsigned short, int> labelsCountInBox;

	for (int i = 0; i < labelImage.rows; ++i)
	{
		for (int j = 0; j < labelImage.cols; ++j)
		{
			++totalLabelsCount[labelImage.at<ushort>(i, j)];
		}
	}

	int minCol = std::min(beginPoint.x, endPoint.x);
	int maxCol = std::max(beginPoint.x, endPoint.x);
	int minRow = std::min(beginPoint.y, endPoint.y);
	int maxRow = std::max(beginPoint.y, endPoint.y);
	minCol = std::max(minCol, 0);
	maxCol = std::min(maxCol, labelImage.cols);
	minRow = std::max(minRow, 0);
	maxRow = std::min(maxRow, labelImage.rows);

	std::cout << "Row range: " << minRow << " " << maxRow << std::endl;
	std::cout << "Col range: " << minCol << " " << maxCol << std::endl;

	for (int i = minRow; i < maxRow; ++i)
	{
		for (int j = minCol; j < maxCol; ++j)
		{
			++labelsCountInBox[labelImage.at<ushort>(i, j)];
		}
	}

	for (auto count : labelsCountInBox)
	{
		if (count.second > paraMinPatchSize && static_cast<double>(count.second) / totalLabelsCount[count.first] > paraMinRecognizedProportion)
		{
			recognizedPatchesByYolo.push_back(count.first);
		}
	}

	for (auto patch : recognizedPatchesByYolo)
	{
		std::cout << "Recognized patch: " << patch << std::endl;
	}
}

void YoloWrapper::onMouseCallback(int event, int x, int y, int flags, void * userData)
{
	YoloWrapper * yoloWrapper = reinterpret_cast<YoloWrapper *>(userData);
	yoloWrapper->onMouseCallback(event, x, y);
}

void YoloWrapper::onMouseCallback(int event, int x, int y)
{
	switch (event)
	{
		case cv::EVENT_LBUTTONDOWN:
			beginDrawingBox = true;
			finishDrawingBox = false;
			beginPoint.x = x;
			beginPoint.y = y;
			endPoint.x = x;
			endPoint.y = y;
			break;
		case cv::EVENT_MOUSEMOVE:
			if (beginDrawingBox)
			{
				endPoint.x = x;
				endPoint.y = y;
			}
			break;
		case cv::EVENT_LBUTTONUP:
			beginDrawingBox = false;
			finishDrawingBox = true;
			endPoint.x = x;
			endPoint.y = y;
		default:
			break;
	}
}
