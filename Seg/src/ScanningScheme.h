#ifndef SCANNING_SCHEME_H_
#define SCANNING_SCHEME_H_

#include <Eigen/Dense>
#include <vector>
#include <array>
#include <map>
#include <string>
#include <queue>
#include <mutex>
#include <pangolin/pangolin.h>

#include "PatchFusion.h"
#include "ModelDataset.h"
#include "VisualWord.h"
#include "PartialMatchingFacada.h"
#include "NBO.h"
#include "NBV.h"
#include "Shaders/Shaders.h"

struct Patch
{
	int num;
	Eigen::Vector3f * positions;
	Eigen::Vector3f * colors;
	Eigen::Vector3f * normals;
	float * radii;
	int index;
};

class ScanningScheme
{
	public:
		ScanningScheme() = default;
		ScanningScheme(const std::string & datasetDir, const std::string & plyDir);
		ScanningScheme(const std::string & datasetDir, const std::string & visualWordFilePath, const std::string & plyDir);
		~ScanningScheme();

		void process(int pointNum,
					 Eigen::Vector4f * mapData,
					 float confidence,
					 const Eigen::Matrix4f & camPose,
					 std::vector<unsigned short> recognizedPatchesByYolo,
					 double probability);

		void process(const Eigen::Matrix4f & camPose,
					 std::vector<unsigned short> recognizedPatchesByYolo,
					 double probability);

		void loadPatchData(const std::string & patchDir);

		void clearGoal();
		void clearRecognizedUnmergedPatches();

		void renderNBOMatchingModels(pangolin::OpenGlMatrix mvp);

		void saveRecognizedObjects(const std::string & saveDir);

		// SET functions
		void setProcessingTrue() { processing = true; }

		// GET functions
		const std::queue<std::array<float, 6>> & getNBVGoals() const { return nbvGoalsQueue; }
		const std::vector<std::vector<unsigned short>> & getRecognizedUnmergedPatchLabels() { mtx.lock(); return recognizedUnmergedPatchLabels; }
		const SegmentedPatch & getNBOPatch() const { return nbo.getNBO(); }
		const std::vector<unsigned short> & getGroundPlainLabels() const { return groundPlainLabels; }

		// IS functions
		bool hasGoal() const { return goalExistence; }
		bool hasRecognizedUnmergedPatches() const { return recognizedUnmergedPatchLabels.size() != 0; }
		bool isProcessing() const { return processing; }

	private:
		void processNBO(const Eigen::Matrix4f & camPose,
						const std::vector<unsigned short> & recognizedPatchesByYolo,
						double probability,
						bool searchFar = false);

		void decomposeMapIntoPatches(int pointNum, Eigen::Vector4f * mapData, float confidence);

		void removeInappropriatePatches(const Eigen::Matrix4f & camPose);
		void findGroundPlainPatches();
		bool isRecognizedPatch(const SegmentedPatch & patch);
		bool isBlacklistedPatch(const SegmentedPatch & patch);
		bool isGroundPlainPatch(const SegmentedPatch & patch);

		void insertRecognizedObjects(const std::vector<SegmentedPatch> & objects);

		void computeNBVPose();

		void loadNBOMatchingModels(const std::vector<std::string> & modelNames, const std::vector<Eigen::Matrix4f> & transMats);
		void copyNBOMatchingModelsIntoGL();

		void updateBlacklist(const Eigen::Vector3f & nboCenterPosition);

		std::vector<SegmentedPatch> patchData;
		std::vector<SegmentedPatch> farPatches;
		std::vector<SegmentedPatch> nboMatchingModels;
		std::map<unsigned short, SegmentedPatch> recognizedObjects;
		std::vector<std::vector<unsigned short>> recognizedUnmergedPatchLabels;
		std::vector<unsigned short> groundPlainLabels;

		Eigen::Vector3f lastNBOPosition;
		int nboCounter = 0;
		std::vector<Eigen::Vector3f> positionBlacklist;

		int paraMinPatchPointNum = 500;
		float paraFarPatchDis = 5.0;
		double paraMaxErrorConsiderRecognized = 0.012;
		int paraBlacklistCountThreshold = 5;
		float paraBlacklistDistanceThreshold = 1.0;
		double paraMaxPatchDiameter = 3.0;

		std::shared_ptr<PartialMatchingFacada> partialMatchingFacada;
		PatchFusion patchFusion;
		NBO nbo;

		std::string modelsPlyDir;

		std::queue<std::array<float, 6>> nbvGoalsQueue;
		bool goalExistence = false;
		bool alreadyCopy = false;
		bool processing = false;

        std::shared_ptr<Shader> drawProgram;
		GLuint matchedModelsVBO = 0;
		int totalModelsPointNum = 0;

		std::mutex mtx;
};

#endif /* SCANNING_SCHEME_H_ */
