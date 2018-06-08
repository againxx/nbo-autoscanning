#include "PlyWriter.h"

bool PlyWriter::write(int pointNum,
					  const Eigen::Vector3f * positions,
					  const Eigen::Vector3f * colors,
					  const Eigen::Vector3f * normals,
					  const float * radii,
					  bool ascii)
{
	isASCII = ascii;
	hasColor = colors ? true : false;
	hasNormal = normals ? true : false;
	hasRadius = radii ? true : false;

	file.open(filePath);
	try
	{
		if (!file)
		{
			throw std::runtime_error("Error: couldn't open file!");
		}
		writeHeader(pointNum);
		writeBody(pointNum, positions, colors, normals, radii);
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

void PlyWriter::writeHeader(int pointNum)
{
	if (pointNum == 0)
	{
		throw std::runtime_error("Error: point num is zero!");
		return;
	}

	file << "ply" << std::endl;
	if (isASCII)
		file << "format ascii 1.0" << std::endl;
	else
		file << "format binary_little_endian 1.0" << std::endl;

	file << "element vertex " << pointNum << std::endl;
	file << "property float x" << std::endl;
	file << "property float y" << std::endl;
	file << "property float z" << std::endl;

	if (hasColor)
	{
		file << "property uchar red" << std::endl;
		file << "property uchar green" << std::endl;
		file << "property uchar blue" << std::endl;
	}

	if (hasNormal)
	{
		file << "property float nx" << std::endl;
		file << "property float ny" << std::endl;
		file << "property float nz" << std::endl;
	}

	if (hasRadius)
		file << "property float radius" << std::endl;

	file << "end_header" << std::endl;
}

void PlyWriter::writeBody(int pointNum,
						  const Eigen::Vector3f * positions,
						  const Eigen::Vector3f * colors,
						  const Eigen::Vector3f * normals,
						  const float * radii)
{
	if (isASCII)
		writeAsciiData(pointNum, positions, colors, normals, radii);
	else
		writeBinaryData(pointNum, positions, colors, normals, radii);
}

void PlyWriter::writeAsciiData(int pointNum,
							   const Eigen::Vector3f * positions,
							   const Eigen::Vector3f * colors,
							   const Eigen::Vector3f * normals,
							   const float * radii)
{
	for (int i = 0; i < pointNum; ++i)
	{
		file << positions[i][0];
		file << " " << positions[i][1];
		file << " " << positions[i][2];

		if (hasColor)
		{
			file << " " << static_cast<int>(colors[i][0]);
			file << " " << static_cast<int>(colors[i][1]);
			file << " " << static_cast<int>(colors[i][2]);
		}

		if (hasNormal)
		{
			file << " " << -normals[i][0];
			file << " " << -normals[i][1];
			file << " " << -normals[i][2];
		}

		if (hasRadius)
			file << " " << radii[i];

		file << std::endl;
	}
}

void PlyWriter::writeBinaryData(int pointNum,
								const Eigen::Vector3f * positions,
								const Eigen::Vector3f * colors,
								const Eigen::Vector3f * normals,
								const float * radii)
{
	file.close();
	file.open(filePath, std::ios::app | std::ios::binary);

    for(int i = 0; i < pointNum; ++i)
    {
		float value;
		value = positions[i][0];
		file.write(reinterpret_cast<const char*>(&value), sizeof(float));

		value = positions[i][1];
		file.write(reinterpret_cast<const char*>(&value), sizeof(float));

		value = positions[i][2];
		file.write(reinterpret_cast<const char*>(&value), sizeof(float));

		if (hasColor)
		{
			unsigned char r = static_cast<int>(colors[i][0]);
			unsigned char g = static_cast<int>(colors[i][1]);
			unsigned char b = static_cast<int>(colors[i][2]);

			file.write(reinterpret_cast<const char*>(&r), sizeof(unsigned char));
			file.write(reinterpret_cast<const char*>(&g), sizeof(unsigned char));
			file.write(reinterpret_cast<const char*>(&b), sizeof(unsigned char));
		}

		if (hasNormal)
		{
			value = -normals[i][0];
			file.write(reinterpret_cast<const char*>(&value), sizeof(float));

			value = -normals[i][1];
			file.write(reinterpret_cast<const char*>(&value), sizeof(float));

			value = -normals[i][2];
			file.write(reinterpret_cast<const char*>(&value), sizeof(float));
		}

		if (hasRadius)
		{
			value = radii[i];
			file.write(reinterpret_cast<const char*>(&value), sizeof(float));
		}
    }
}
