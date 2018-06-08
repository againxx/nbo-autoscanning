#ifndef BOUNDING_BOX_H_
#define BOUNDING_BOX_H_

#include <Eigen/Dense>
#include <limits>
#include <algorithm>

class BoundingBox
{
	public:
		BoundingBox() = default;
		BoundingBox(double xmax, double xmin, double ymax, double ymin, double zmax, double zmin);
		~BoundingBox() = default;

		void reset();
		void reset(const Eigen::Vector3f * pointPositions, int pointNum);
		void reset(const float * pointPositions, int pointNum);
		
		double getDiameter() const;

		friend BoundingBox operator+(const BoundingBox & bd1, const BoundingBox & bd2);

	public:
		double xMax = -std::numeric_limits<double>::max();
		double xMin =  std::numeric_limits<double>::max();
		double yMax = -std::numeric_limits<double>::max();
		double yMin =  std::numeric_limits<double>::max();
		double zMax = -std::numeric_limits<double>::max();
		double zMin =  std::numeric_limits<double>::max();
};

BoundingBox operator+(const BoundingBox & bd1, const BoundingBox & bd2);

#endif /* ifndef BOUNDING_BOX_H_ */
