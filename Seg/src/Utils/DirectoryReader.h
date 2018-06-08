#ifndef DIRECTORY_READER_H_
#define DIRECTORY_READER_H_

#include <iostream>
#include <vector>
#include <string>
#include <dirent.h>
#include <sys/stat.h>

class DirectoryReader
{
	public:
		DirectoryReader(const std::string & dirPath);
		~DirectoryReader() = default;

		// GET functions
		bool getNextFile(std::string & name, std::string & path)
		{
			if (fileIndex == fileNum)
				return false;
			else
			{
				name = fileNames[fileIndex];
				path = filePaths[fileIndex];
				++fileIndex;
				return true;
			}
		}

		// IS functions
		bool isEmpty() const { return fileNum == 0; }
		bool isFinished() const { return fileIndex == fileNum ? true : false; }

	private:
		void read(const std::string & dirPath);

		int fileNum = 0;
		int fileIndex = 0;
		std::vector<std::string> fileNames; 	
		std::vector<std::string> filePaths; 	
};

#endif /* DIRECTORY_READER_H_ */
