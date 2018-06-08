#ifndef FILE_NAME_PARSER_H_
#define FILE_NAME_PARSER_H_

#include <string>

class FileNameParser
{
	public:
		FileNameParser(const std::string & name)
		 : fileName(name)
		{
			extractModelName();
			extractObjectName();
			extractCategoryName();
			extractPostfix();
		}

		~FileNameParser() = default;

		// GET functions
		const std::string & getModelName() const { return modelName; }
		const std::string & getObjectName() const { return objectName; }
		const std::string & getCategoryName() const { return categoryName; }
		const std::string & getPostfix() const { return postfix; }

	private:
		void extractModelName()
		{
			auto foundPlace = fileName.find_first_of(".");
			modelName = fileName.substr(0, foundPlace);
		}

		void extractObjectName()
		{
			auto foundPlace = fileName.find_last_of("_");
			objectName = fileName.substr(0, foundPlace);
		}

		void extractCategoryName()
		{
			auto foundPlace = fileName.find_first_of("_");
			categoryName = fileName.substr(0, foundPlace);
		}

		void extractPostfix()
		{
			auto foundPlace = fileName.find_first_of(".");
			postfix = fileName.substr(foundPlace + 1);
		}

		std::string fileName;
		std::string modelName;
		std::string objectName;
		std::string categoryName;
		std::string postfix;
};

#endif /* FILE_NAME_PARSER_H_ */
