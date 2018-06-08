#ifndef KEYPOINT_REPRESENTATION_H_
#define KEYPOINT_REPRESENTATION_H_

#include <Eigen/Dense>
#include <string>
#include <vector>
#include <set>
#include <iostream>
#include <fstream>
#include <memory>

class KeypointRepresentation
{
	public:
		KeypointRepresentation() = default;
		KeypointRepresentation(unsigned int keyptNum, 
							   std::shared_ptr<Eigen::Vector3f> keyptPos, 
							   std::shared_ptr<Eigen::Vector3f> keyptNor, 
							   std::shared_ptr<float> keyptDesc, 
							   std::shared_ptr<std::set<std::pair<int, int>>> keyptNeigh,
							   const Eigen::Vector3f & center,
							   float pcDiameter,
							   const std::string & modelName = "");

		~KeypointRepresentation();

		double computeHistogramDistance(const KeypointRepresentation & otherKeyptRepr) const;

		void reset();

		void load(const std::string & filePath);

		// SET functions
		void setName(const std::string & modelName) { name = modelName; }
		void setClusterLabels(const std::vector<int> & labels, int num);

		// GET functions
		const std::string & 	 getName() const { return name; }
		const Eigen::Vector3f *  getPositions() const { return positions.get(); }
		const Eigen::Vector3f *  getNormals() const { return normals.get(); }
		const float * 			 getDescriptors() const { return descriptors.get(); }
		const Eigen::Vector3f &  getCenterPosition() const { return centerPosition; }
		const std::vector<int> & getClusterLabels() const { return clusterLabels; }
		const Eigen::VectorXd &  getClusterLabelHistogram() const { return clusterLabelHistogram; }
		const Eigen::MatrixXd &  getNeighborLabelHistogram() const { return neighborLabelHistogram; }
		float 					 getAverageRadius() const { return averageRadius; }
		float 					 getAverageLength() const { return averageLength; }
		float 					 getDiameter() const { return diameter; }
		unsigned int 			 getNum() const { return num; }
		int 					 getClusterNum() const { return clusterNum; }

		// IS functions
		bool hasNeighborhoods() const { return !neighborhoods->empty(); }
		bool isNeighbor(int keyptId1, int keyptId2) const { return neighborhoods->find({keyptId1, keyptId2}) == neighborhoods->cend() ? false : true; }
		bool isEmpty() const { return num == 0; }

		static constexpr int descriptorSize = 512;

	private:
		void computeAverageRadius();
		void computeAverageLength();

		void computeClusterLabelHistogram();
		void computeNeighborLabelHistogram();

		std::string name;
		unsigned int num = 0;

		std::shared_ptr<Eigen::Vector3f> 			   positions;
		std::shared_ptr<Eigen::Vector3f> 			   normals;
		std::shared_ptr<float>			 			   descriptors;
		std::shared_ptr<std::set<std::pair<int, int>>> neighborhoods;

		int clusterNum;
		std::vector<int> clusterLabels;
		Eigen::VectorXd clusterLabelHistogram;
		Eigen::MatrixXd neighborLabelHistogram;	

		Eigen::Vector3f centerPosition;
		float averageRadius = 0;
		float averageLength = 0;
		float diameter = 0;

		friend std::istream & operator>>(std::istream & is, KeypointRepresentation & keyptRepr);
		friend std::ostream & operator<<(std::ostream & os, const KeypointRepresentation & keyptRepr);
};

std::istream & operator>>(std::istream & is, KeypointRepresentation & keyptRepr);
std::ostream & operator<<(std::ostream & os, const KeypointRepresentation & keyptRepr);

#endif /* ifndef KEYPOINT_REPRESENTATION_H_ */
