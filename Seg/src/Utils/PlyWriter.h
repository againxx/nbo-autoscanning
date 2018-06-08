#ifndef PLY_WRITER_H_
#define PLY_WRITER_H_

#include <iostream>
#include <fstream>
#include <string>
#include <stdexcept>
#include <Eigen/Dense>

class PlyWriter
{
	public:
		PlyWriter() = default;
		PlyWriter(const std::string & fPath) : filePath(fPath) {}
		~PlyWriter() = default;

		bool write(int pointNum,
				   const Eigen::Vector3f * positions,
				   const Eigen::Vector3f * colors,
				   const Eigen::Vector3f * normals,
				   const float * radii,
				   bool ascii = true);

		// SET functions
		void setFilePath(const std::string & fPath) { filePath = fPath; }

	private:
		void writeHeader(int pointNum);

		void writeBody(int pointNum,
					  const Eigen::Vector3f * positions,
					  const Eigen::Vector3f * colors,
					  const Eigen::Vector3f * normals,
					  const float * radii);

		void writeAsciiData(int pointNum,
						   const Eigen::Vector3f * positions,
						   const Eigen::Vector3f * colors,
						   const Eigen::Vector3f * normals,
						   const float * radii);

		void writeBinaryData(int pointNum,
							const Eigen::Vector3f * positions,
							const Eigen::Vector3f * colors,
							const Eigen::Vector3f * normals,
							const float * radii);

		std::ofstream file;
		std::string filePath;

		bool isASCII = true;
		bool hasColor = false;
		bool hasNormal = false;
		bool hasRadius = false;
};

#endif /* PLY_WRITER_H_ */
