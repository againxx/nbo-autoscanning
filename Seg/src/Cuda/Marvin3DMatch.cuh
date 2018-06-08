#ifndef MARVIN_3DMATCH_H_
#define MARVIN_3DMATCH_H_

#include <string>
#include "marvin.hpp"

class Marvin3DMatch
{
	public:
		static const Marvin3DMatch & getInstance()
		{
			static Marvin3DMatch instance;
			return instance;
		}

		~Marvin3DMatch()
		{
			delete convolutionalNet;
		}

		marvin::Net * getConvolutionalNet() const { return convolutionalNet; }

	private:
		Marvin3DMatch()
		{
			std::string homePath(getenv("HOME"));
			convolutionalNet = new marvin::Net(homePath + "/iroboscan/Seg/src/Cuda/3dmatch-net-test.json");
			convolutionalNet->Malloc(marvin::Testing);
			convolutionalNet->loadWeights(homePath + "/iroboscan/Seg/src/Cuda/3dmatch-weights-snapshot-137000.marvin");
		}

		marvin::Net * convolutionalNet = nullptr;
		
};

#endif /* MARVIN_3DMATCH_H_ */
