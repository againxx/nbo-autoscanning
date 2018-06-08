#ifndef PLY_READER_H_
#define PLY_READER_H_

#include <iostream>
#include <fstream>
#include <string>
#include <stdexcept>
#include <Eigen/Dense>

class PlyReader
{
	public:
		PlyReader() = default;
		PlyReader(const std::string & fPath) : filePath(fPath) {}
		~PlyReader() = default;

		bool read(int & pointNum,
				  Eigen::Vector3f* & positions,
				  Eigen::Vector3f* & colors,
				  Eigen::Vector3f* & normals,
				  float* & radii);

		// SET functions
		void setFilePath(const std::string & fPath) { filePath = fPath; }

	private:
		void analyzeHeader(int & pointNum);

		void readBody(int & pointNum,
					  Eigen::Vector3f* & positions,
					  Eigen::Vector3f* & colors,
					  Eigen::Vector3f* & normals,
					  float* & radii);

		int readAsciiData(int pointNum,
						   Eigen::Vector3f* & positions,
						   Eigen::Vector3f* & colors,
						   Eigen::Vector3f* & normals,
						   float* & radii);

		int readBinaryData(int pointNum,
							Eigen::Vector3f* & positions,
							Eigen::Vector3f* & colors,
							Eigen::Vector3f* & normals,
							float* & radii);
		
		std::ifstream file;
		std::string filePath;

		bool isASCII = true;
		bool hasColor = false;
		bool hasNormal = false;
		bool hasRadius = false;
};

#endif /* PLY_READER_H_ */
