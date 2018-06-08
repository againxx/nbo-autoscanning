#ifndef NBO_H_
#define NBO_H_

#include <Eigen/Dense>
#include "SegmentedPatch.h"
#include "PartialMatchingFacada.h"

class NBO
{
	public:
		NBO(std::shared_ptr<PartialMatchingFacada> pmFacada);
		~NBO() = default;

		void computeNBO(const Eigen::Matrix4f & currPose, const std::vector<SegmentedPatch> & objectHypo);

		// GET functions
		const SegmentedPatch & getNBO() const { return nbo; }
		const MatchingResultParser & getNBOMatchingResult() const { return nboMatchingResult; }
		const std::vector<SegmentedPatch> & getRecognizedObjects() const { return recognizedObjects; }

		// IS functions
		bool hasNBO() const { return nboFlag; }

	  private:
		inline double gaussFunction(double x);

		std::shared_ptr<PartialMatchingFacada> partialMatchingFacada;

		SegmentedPatch nbo;
		MatchingResultParser nboMatchingResult;
		bool nboFlag = false;

		std::vector<SegmentedPatch> recognizedObjects;

		// Parameters
		const double paraObjectnessWeight = 1.0;
		const double paraPositionWeight = 1.5;
		const double paraOrientWeight = 1.0;
		const double paraSizeWeight = 1.0;
		const double paraGaussFunctionCenter = 0.0;
		const double paraGaussFunctionWidth = 0.05;
		const double paraMaxErrorConsiderRecognized = 0.012;
};

#endif /* NBO_H_ */
