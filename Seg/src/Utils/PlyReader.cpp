#include "PlyReader.h"

bool PlyReader::read(int & pointNum,
					 Eigen::Vector3f* & positions,
					 Eigen::Vector3f* & colors,
					 Eigen::Vector3f* & normals,
					 float* & radii)
{
	file.open(filePath.c_str());
	try
	{
		if (!file)
		{
			throw std::runtime_error("Error: ply file not found!");
		}
		analyzeHeader(pointNum);
		readBody(pointNum, positions, colors, normals, radii);
	}
	catch (std::runtime_error err)
	{
		std::cerr << err.what() << std::endl;
		file.close();
		return false;
	}

	file.close();
	return true;
}

void PlyReader::analyzeHeader(int & pointNum)
{
	std::string lineStr;
	std::getline(file, lineStr);
	if (lineStr != "ply")
		throw std::runtime_error("Error: this is not a ply file!");

	std::getline(file, lineStr);
	if (lineStr.find("ascii") == std::string::npos)
	{
		if (lineStr.find("binary") == std::string::npos)
			throw std::runtime_error("Error: wrong file format!");
		else
			isASCII = false;
	}
	else
		isASCII = true;

	pointNum = 0;
	bool finishHead = false;
	bool hasPosition = false;
	while (!finishHead)
	{
		std::getline(file, lineStr);

		if (!hasPosition)
		{
			if (lineStr.find("float x") != std::string::npos)
				hasPosition = true;
		}
		if (!hasColor)
		{
			if (lineStr.find("uchar red") != std::string::npos)
				hasColor = true;
		}
		if (!hasNormal)
		{
			if (lineStr.find("float nx") != std::string::npos)
				hasNormal = true;
		}
		if (!hasRadius)
		{
			if (lineStr.find("float radius") != std::string::npos)
				hasRadius = true;
		}
		if (pointNum == 0)
		{
			if (lineStr.find("vertex") != std::string::npos)
			{
				std::istringstream tempLine(lineStr);
				std::string tempLinePrefix;
				tempLine >> tempLinePrefix;
				tempLine >> tempLinePrefix;
				tempLine >> pointNum;
			}
		}
		if (lineStr.find("end") != std::string::npos)
			finishHead = true;
	}

	if (!hasPosition)
		throw std::runtime_error("Error: this ply file doesn't have a position property!");
}

void PlyReader::readBody(int & pointNum,
						 Eigen::Vector3f* & positions,
						 Eigen::Vector3f* & colors,
						 Eigen::Vector3f* & normals,
						 float* & radii)
{
	if (pointNum == 0)
		throw std::runtime_error("Error: empty ply file!");

	positions = new Eigen::Vector3f[pointNum];
	colors = hasColor ? new Eigen::Vector3f[pointNum] : nullptr;
	normals = hasNormal ? new Eigen::Vector3f[pointNum] : nullptr;
	radii = hasRadius ? new float[pointNum] : nullptr;

	int realPointNum = 0;	// point num wipe off nan
	if (isASCII)
		realPointNum = readAsciiData(pointNum, positions, colors, normals, radii);
	else
		realPointNum = readBinaryData(pointNum, positions, colors, normals, radii);

	if (realPointNum != pointNum)
	{
		Eigen::Vector3f * tempPositions = positions;
		positions = new Eigen::Vector3f[realPointNum];
		memcpy(positions, tempPositions, sizeof(Eigen::Vector3f) * realPointNum);
		delete [] tempPositions;

		if (hasColor)
		{
			Eigen::Vector3f * tempColors = colors;
			colors = new Eigen::Vector3f[realPointNum];
			memcpy(colors, tempColors, sizeof(Eigen::Vector3f) * realPointNum);
			delete [] tempColors;
		}

		if (hasNormal)
		{
			Eigen::Vector3f * tempNormals = normals;
			normals = new Eigen::Vector3f[realPointNum];
			memcpy(normals, tempNormals, sizeof(Eigen::Vector3f) * realPointNum);
			delete [] tempNormals;
		}

		if (hasRadius)
		{
			float * tempRadii = radii;
			radii = new float[realPointNum];
			memcpy(radii, tempRadii, sizeof(float) * realPointNum);
			delete [] tempRadii;
		}
		pointNum = realPointNum;
	}
}

int PlyReader::readAsciiData(int pointNum,
							  Eigen::Vector3f* & positions,
							  Eigen::Vector3f* & colors,
							  Eigen::Vector3f* & normals,
							  float* & radii)
{
	int realPointNum = pointNum;
	std::string lineStr;
	for (int i = 0; i < realPointNum; )
	{
		std::getline(file, lineStr);
		if (lineStr.find("nan") != std::string::npos)
		{
			--realPointNum;
			continue;
		}

		std::istringstream tempLine(lineStr);

		tempLine >> positions[i][0];
		tempLine >> positions[i][1];
		tempLine >> positions[i][2];

		if (hasColor)
		{
			int tempColor;
			tempLine >> tempColor;
			colors[i][0] = tempColor;
			tempLine >> tempColor;
			colors[i][1] = tempColor;
			tempLine >> tempColor;
			colors[i][2] = tempColor;
		}

		if (hasNormal)
		{
			tempLine >> normals[i][0];
			tempLine >> normals[i][1];
			tempLine >> normals[i][2];
			normals[i] *= -1;
		}

		if (hasRadius)
			tempLine >> radii[i];

		++i;
	}
	return realPointNum;
}

int PlyReader::readBinaryData(int pointNum,
							  Eigen::Vector3f* & positions,
							  Eigen::Vector3f* & colors,
							  Eigen::Vector3f* & normals,
							  float* & radii)
{
	int realPointNum = pointNum;
	for (int i = 0; i < realPointNum; )
	{
		file.read(reinterpret_cast<char *>(&positions[i][0]), sizeof(float));
		file.read(reinterpret_cast<char *>(&positions[i][1]), sizeof(float));
		file.read(reinterpret_cast<char *>(&positions[i][2]), sizeof(float));

		if (hasColor)
		{
			unsigned char tempColor;
			file.read(reinterpret_cast<char *>(&tempColor), sizeof(unsigned char));
			colors[i][0] = tempColor;
			file.read(reinterpret_cast<char *>(&tempColor), sizeof(unsigned char));
			colors[i][1] = tempColor;
			file.read(reinterpret_cast<char *>(&tempColor), sizeof(unsigned char));
			colors[i][2] = tempColor;
		}

		if (hasNormal)
		{
			file.read(reinterpret_cast<char *>(&normals[i][0]), sizeof(float));
			file.read(reinterpret_cast<char *>(&normals[i][1]), sizeof(float));
			file.read(reinterpret_cast<char *>(&normals[i][2]), sizeof(float));
			if (std::isnan(normals[i][0]) || std::isnan(normals[i][1]) || std::isnan(normals[i][2]))
			{
				--realPointNum;
				continue;
			}
			normals[i] *= -1;
		}

		if (hasRadius)
			file.read(reinterpret_cast<char *>(&radii[i]), sizeof(float));

		++i;
	}
	return realPointNum;
}
