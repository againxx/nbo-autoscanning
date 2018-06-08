#include "DirectoryReader.h"

DirectoryReader::DirectoryReader(const std::string & dirPath)
{
    if (dirPath.back() == '/')
        read(dirPath.substr(0, dirPath.size() - 1));
    else
	    read(dirPath);
}

void DirectoryReader::read(const std::string & dirPath)
{
	if (dirPath.empty())
	{
		std::cerr << "Error: directory path is null!" << std::endl; 
		return;
	}

	// Open a dir with some files
	DIR * dir = opendir(dirPath.c_str());
	if (dir == nullptr)
	{
		std::cerr << "Error: directory path is not a valid directory." << std::endl;
		closedir(dir);
		return;
	}

	struct stat fileStat;
	struct dirent * dirEntry;
	std::string filePath;
	std::string fileName;

	// Go through files in dir
	while ((dirEntry = readdir(dir)))
	{
		fileName = dirEntry->d_name;
		filePath = dirPath + "/" + fileName;

		// If the file is a directory (or in some way invalid) we'll skip it
		if (stat(filePath.c_str(), &fileStat))
			continue;
		if (S_ISDIR(fileStat.st_mode))
			continue;

		fileNames.push_back(fileName);
		filePaths.push_back(filePath);
		++fileNum;
	}
	closedir(dir);
}
