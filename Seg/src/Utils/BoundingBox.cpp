#include "BoundingBox.h"

BoundingBox::BoundingBox(double xmax, double xmin, double ymax, double ymin, double zmax, double zmin)
 : xMax(xmax),
   xMin(xmin),
   yMax(ymax),
   yMin(ymin),
   zMax(zmax),
   zMin(zmin)
{ 

}

void BoundingBox::reset()
{
	xMin = yMin = zMin = std::numeric_limits<double>::max();
	xMax = yMax = zMax = -std::numeric_limits<double>::max();
}

void BoundingBox::reset(const Eigen::Vector3f * pointPositions, int pointNum)
{
	xMin = yMin = zMin = std::numeric_limits<double>::max();
	xMax = yMax = zMax = -std::numeric_limits<double>::max();

	// Compute bounding box
	for (int i = 0; i < pointNum; ++i)
	{
		xMin = std::min(xMin, (double)pointPositions[i][0]);
		yMin = std::min(yMin, (double)pointPositions[i][1]);
		zMin = std::min(zMin, (double)pointPositions[i][2]);
		xMax = std::max(xMax, (double)pointPositions[i][0]);
		yMax = std::max(yMax, (double)pointPositions[i][1]);
		zMax = std::max(zMax, (double)pointPositions[i][2]);
	}
}

void BoundingBox::reset(const float * pointPositions, int pointNum)
{
	xMin = yMin = zMin = std::numeric_limits<double>::max();
	xMax = yMax = zMax = -std::numeric_limits<double>::max();

	// Compute bounding box
	for (int i = 0; i < pointNum; ++i)
	{
		xMin = std::min(xMin, (double)pointPositions[3 * i]);
		yMin = std::min(yMin, (double)pointPositions[3 * i + 1]);
		zMin = std::min(zMin, (double)pointPositions[3 * i + 2]);
		xMax = std::max(xMax, (double)pointPositions[3 * i]);
		yMax = std::max(yMax, (double)pointPositions[3 * i + 1]);
		zMax = std::max(zMax, (double)pointPositions[3 * i + 2]);
	}
}

double BoundingBox::getDiameter() const
{
	double diameter = std::max(xMax - xMin, yMax - yMin);
	diameter = std::max(zMax - zMin, diameter);
	return diameter;
}

BoundingBox operator+(const BoundingBox & bd1, const BoundingBox & bd2)
{
	double newXMax, newXMin, newYMax, newYMin, newZMax, newZMin;
	newXMax = bd1.xMax > bd2.xMax ? bd1.xMax : bd2.xMax;
	newXMin = bd1.xMin < bd2.xMin ? bd1.xMin : bd2.xMin;
	newYMax = bd1.yMax > bd2.yMax ? bd1.yMax : bd2.yMax;
	newYMin = bd1.yMin < bd2.yMin ? bd1.yMin : bd2.yMin;
	newZMax = bd1.zMax > bd2.zMax ? bd1.zMax : bd2.zMax;
	newZMin = bd1.zMin < bd2.zMin ? bd1.zMin : bd2.zMin;

	BoundingBox newBoundingBox(newXMax, newXMin, newYMax, newYMin, newZMax, newZMin);
	return newBoundingBox;
}
