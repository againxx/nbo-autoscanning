#include "PartialCorrespondence.h"

PartialCorrespondence::PartialCorrespondence(const KeypointRepresentation & firstKeyptRepr, const KeypointRepresentation & secondKeyptRepr)
 : firstKeypointRepresentation(firstKeyptRepr),
   secondKeypointRepresentation(secondKeyptRepr)
{
	clusterFirstKeypointIndexes();
}

void PartialCorrespondence::process()
{
	initializeRedundantCorrespondence();
	pruneRedundantCorrespondence();
}

void PartialCorrespondence::initializeRedundantCorrespondence()
{
	redundantCorrespondence.clear();
	for (size_t i = 0; i < secondKeypointRepresentation.getNum(); ++i)
	{
		int assignmentNum = secondKeypointRepresentation.getClusterLabels().size() / secondKeypointRepresentation.getNum();
		int clusterIndex = secondKeypointRepresentation.getClusterLabels()[assignmentNum * i];

		auto pairComp =
			[](const std::pair<int, double> & p1, const std::pair<int, double> & p2)
			{ return p1.second > p2.second; };

		std::priority_queue<std::pair<int, double>,
							std::vector<std::pair<int, double>>,
							decltype(pairComp)> descDisPriorityQueue(pairComp);

		for (size_t j = 0; j < clusteredFirstKeypointIndexes[clusterIndex].size(); ++j)
		{
			int keyptIndex = clusteredFirstKeypointIndexes[clusterIndex][j];

			double distance = 0;
			for (int k = 0; k < KeypointRepresentation::descriptorSize; ++k)
			{
				distance += std::abs(secondKeypointRepresentation.getDescriptors()[i * KeypointRepresentation::descriptorSize + k]
						- firstKeypointRepresentation.getDescriptors()[keyptIndex * KeypointRepresentation::descriptorSize + k]);
			}
			if (distance > paraDescriptorDisThreshold)
				continue;
			descDisPriorityQueue.emplace(keyptIndex, distance);
		}

		for (int j = 0; j < paraRedundantNum; ++j)
		{
			if (!descDisPriorityQueue.empty() && descDisPriorityQueue.top().second < paraDescriptorDisThreshold)
			{
				redundantCorrespondence.emplace_back(descDisPriorityQueue.top().first, i);
				descDisPriorityQueue.pop();
			}
			else
				break;
		}
	}
}

void PartialCorrespondence::pruneRedundantCorrespondence()
{
	int corrNum = redundantCorrespondence.size();
	Eigen::SparseMatrix<float> K(corrNum, corrNum);
	Eigen::SparseMatrix<float> P(corrNum, corrNum);
	std::vector<Eigen::Triplet<float>> tripleK;
	for (int i = 0; i < corrNum; ++i) 
	{
		for (int j = 0; j < corrNum; ++j) 
		{ 
			if(i == j)
				continue;

			int firstPointIndex1 = redundantCorrespondence[i].first;
			int firstPointIndex2 = redundantCorrespondence[j].first;
			int secondPointIndex1 = redundantCorrespondence[i].second;
			int secondPointIndex2 = redundantCorrespondence[j].second;

			float dis1 = (firstKeypointRepresentation.getPositions()[firstPointIndex1]
				- firstKeypointRepresentation.getPositions()[firstPointIndex2]).norm();

			float dis2 = (secondKeypointRepresentation.getPositions()[secondPointIndex1]
				- secondKeypointRepresentation.getPositions()[secondPointIndex2]).norm();

			float kij = std::min(dis1/dis2, dis2/dis1);

			if (kij >= paraConfidence && dis1 < paraSigma * paraMaxDistance && dis2 < paraSigma * paraMaxDistance)
				tripleK.push_back(Eigen::Triplet<float>(i, j, std::pow((kij - paraConfidence) / (1 - paraConfidence), 2)));
		}
	}
	K.setFromTriplets(tripleK.begin(), tripleK.end());

	std::vector<float> d(corrNum, 0);
	std::vector<std::pair<float, int>> sortedCorrespondence, finalCorrespondence;

	float dSum = 0; 
	for (int i = 0; i < corrNum; ++i)
	{
		for (int j = 0; j < corrNum; ++j) 
		{
			d[i] += K.coeff(i, j);
		}        
		dSum += d[i];    
	}
	
	for (int i = 0; i < corrNum; ++i) 
	{
		if(d[i] != 0)
			sortedCorrespondence.emplace_back(d[i] / dSum, i); 	
	}
	std::sort(sortedCorrespondence.begin(), sortedCorrespondence.end(),
			[](const std::pair<float, int> & p1, const std::pair<float, int> & p2)
			{ return p1.first > p2.first; });

	if (!sortedCorrespondence.empty())
	{
		finalCorrespondence.push_back(sortedCorrespondence.back());
		sortedCorrespondence.pop_back();
	}

	int finalCorrNum = 0;
	while (!sortedCorrespondence.empty() && finalCorrNum < paraMaxCorrespondenceNum)
    {
		auto tempCorr = sortedCorrespondence.back();
		sortedCorrespondence.pop_back();

		int count = 0;
		for (const auto & corrPair : finalCorrespondence)
        {
			int firstPointIndex1 = redundantCorrespondence[tempCorr.second].first;
			int firstPointIndex2 = redundantCorrespondence[corrPair.second].first;
			int secondPointIndex1 = redundantCorrespondence[tempCorr.second].second;
			int secondPointIndex2 = redundantCorrespondence[corrPair.second].second;

			float dis1 = (firstKeypointRepresentation.getPositions()[firstPointIndex1]
				- firstKeypointRepresentation.getPositions()[firstPointIndex2]).norm();

			float dis2 = (secondKeypointRepresentation.getPositions()[secondPointIndex1]
				- secondKeypointRepresentation.getPositions()[secondPointIndex2]).norm();

			if(std::min(dis1 / dis2, dis2 / dis1) >= paraConfidence)
				++count;
		}

		if (count >= 0.6 * finalCorrespondence.size())
		{
			finalCorrespondence.push_back(tempCorr);
			++finalCorrNum;
		}
	}

	prunedCorrespondence.clear();
	for (const auto & corrPair : finalCorrespondence)
	{
		prunedCorrespondence.push_back(redundantCorrespondence[corrPair.second]);
	}
}

void PartialCorrespondence::clusterFirstKeypointIndexes()
{
	clusteredFirstKeypointIndexes.clear();
	clusteredFirstKeypointIndexes.resize(firstKeypointRepresentation.getClusterNum());

	int assignmentNum = firstKeypointRepresentation.getClusterLabels().size() / firstKeypointRepresentation.getNum();
	for (size_t i = 0; i < firstKeypointRepresentation.getNum(); ++i)
	{
		clusteredFirstKeypointIndexes[firstKeypointRepresentation.getClusterLabels()[assignmentNum * i]].push_back(i);
	}
}

//float dis(Eigen::Vector3f a, Eigen::Vector3f b)
//{
//        return (a - b).norm();
//}

//int main()  
//{
//		
//    std::ifstream fs0("../data/clusterCenters.txt");
//    int clusterNum = 50;
//    float clusterCenters[clusterNum][KeypointRepresentation::descriptorSize];
//    for(int i = 0; i < 50; ++i)
//    {
//        for(int j = 0; j < 512; ++j)
//        {
//            fs0 >>  clusterCenters[i][j];
//        }
//    }
//
//    std::ifstream fs1("../data/chair_0005.keypts.txt");
//    KeypointRepresentation keyptRepr1;
//    fs1 >> keyptRepr1;
//
//    std::ifstream fs2("../data/chair_0005_1+3.keypts.txt");
//    KeypointRepresentation keyptRepr2;
//    fs2 >> keyptRepr2;
//
//		
//    double thre = 1,c0 = 0.7, sigma = 0.1, maxDis = 0.8;
//    typedef std::pair<Eigen::Vector3f,Eigen::Vector3f> cpd;
//    std::vector<cpd> cpds; //correspondences
//
//    std::vector<std::vector<int> > label1(50);
//
//    for (int i = 0; i < keyptRepr1.getNum(); ++i)
//    {
//        double min = std::numeric_limits<double>::max();
//        int tag = 0;
//        for (int j = 0; j < clusterNum; ++j)
//        {
//            double dist = 0;
//            for(int k = 0; k < KeypointRepresentation::descriptorSize ; ++k)
//            {
//                dist += std::abs(keyptRepr1.getDescriptors()[i * KeypointRepresentation::descriptorSize + k] - clusterCenters[j][k]);
//            }
//            if(dist < min)
//            {
//                min = dist;
//                tag = j;
//            }
//        }
//        label1[tag].push_back(i);
//    }
//
//    for (int i = 0; i < keyptRepr2.getNum(); ++i)
//    {
//        Eigen::Vector3f point2 = keyptRepr2.getPositions()[i];
//        double min = std::numeric_limits<double>::max();
//        int tag = 0;
//        for (int j = 0; j < clusterNum; ++j)
//        {
//            double dist = 0;
//            for(int k = 0; k < KeypointRepresentation::descriptorSize ; ++k)
//            {
//                dist += std::abs(keyptRepr2.getDescriptors()[i * KeypointRepresentation::descriptorSize + k] - clusterCenters[j][k]);
//            }
//            if(dist < min)
//            {
//                min = dist;
//                tag = j;
//            }
//        }
//
//        double min1 = std::numeric_limits<double>::max(), min2 = std::numeric_limits<double>::max();
//        int tag1 = 0, tag2 = 0;
//        for (int tagj = 0; tagj < label1[tag].size(); ++tagj)
//        {
//            int j = label1[tag][tagj];
//            double dist = 0;
//            for(int k = 0; k < KeypointRepresentation::descriptorSize ; ++k)
//            {
//                dist += std::abs(keyptRepr2.getDescriptors()[i * KeypointRepresentation::descriptorSize + k] - keyptRepr1.getDescriptors()[j * KeypointRepresentation::descriptorSize + k]);
//            }
//            if(dist > thre)
//                continue;
//            if(dist < min1)
//            {
//                min1 = dist;
//                tag1 = j;
//            }
//            else if(dist < min2)
//            {
//                min2 = dist;
//                tag2 = j;
//            }
//        }
//        if(min1 < thre)
//        {
//            Eigen::Vector3f point11 = keyptRepr1.getPositions()[tag1];
//            cpds.push_back(std::pair<Eigen::Vector3f,Eigen::Vector3f>(point2, point11));
//        }
//        if(min2 < thre)
//        {
//            Eigen::Vector3f point12 = keyptRepr1.getPositions()[tag2];
//            cpds.push_back(std::pair<Eigen::Vector3f,Eigen::Vector3f>(point2, point12));
//        }
//    }
//    
//    std::cout<<cpds.size()<<std::endl;
//	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
//    
//	int n = cpds.size();
//	Eigen::SparseMatrix<float> K(n, n);
//	Eigen::SparseMatrix<float> P(n, n);
//	std::vector< Eigen::Triplet<float> > tripleK;
//	std::vector< Eigen::Triplet<float> > tripleP;
//	for (int i = 0; i < n; i++) 
//	{
//    	for (int j = 0; j < n; j++) 
//		{ 
//			if(i == j)
//				continue;
//			float dis1 = dis(cpds[i].first,cpds[j].first);
//			float dis2 = dis(cpds[i].second,cpds[j].second);
//			float kij = std::min(dis1/dis2, dis2/dis1);
//			if(kij>=c0 && dis1<sigma*maxDis && dis2<sigma*maxDis)
//				tripleK.push_back(Eigen::Triplet<float>(i, j, std::pow( (kij - c0)/(1 - c0), 2)));
//      }
//    }
//    K.setFromTriplets(tripleK.begin(), tripleK.end());
//
//	std::vector<float> d(n,0);
//	std::multimap<float, int> C1,C2;
//	float dsum = 0; 
//    for (int i = 0; i < n; i++)
//	{
//        for (int j = 0; j < n; j++) 
//		{
//                d[i] += K.coeffRef(i,j);
//        }        
//       dsum += d[i];    
//    }
//	
//	for (int i = 0; i < n; ++i) 
//	{
//		if(d[i] != 0)
//			C1.insert(std::pair<float, int>(d[i]/dsum,i)); 	
//	}
//
//    std::shared_ptr<pcl::visualization::PCLVisualizer> view(new pcl::visualization::PCLVisualizer("line"));
//    view->setBackgroundColor(0,0,0,0);
//    view->addCoordinateSystem(1.0);
//		int num= 0;
//	 	for (int i = 0; i < keyptRepr1.getNum(); ++i)
//    {
//        Eigen::Vector3f point1 = keyptRepr1.getPositions()[i];
//        pcl::PointXYZRGBA p;
//				p.x = point1[0];
//				p.y = point1[1];
//				p.z = point1[2];
//				p.r = 0;
//				p.g = 0;
//				p.b = 255;
//				p.a = 1;
//				cloud->points.push_back(p);
//	  }
//    for (int j = 0; j < keyptRepr2.getNum(); ++j)
//    {  
//      Eigen::Vector3f point1 = keyptRepr2.getPositions()[j];
//      pcl::PointXYZRGBA p;
//			p.x = point1[0];
//			p.y = point1[1];
//			p.z = point1[2];
//			p.r = 255;
//			p.g = 0;
//			p.b = 0;
//			p.a = 1;
//			cloud->points.push_back(p);
//    }
//    
//    if(!C1.empty())
//    {
//        auto last = C1.end();
//        std::pair<float, int> temp = *(--last);
//        C2.insert(temp);
//        C1.erase(last);
//    }
//
//    while(!C1.empty() && num < 25)
//    {
//        auto last = C1.end();
//        std::pair<float, int> temp = *(--last);
//        C1.erase(last);
//        int a = temp.second;
//
//        pcl::PointXYZRGBA p,q;
//        p.x = cpds[a].first[0];
//        p.y = cpds[a].first[1];
//        p.z = cpds[a].first[2];
//        p.r = 0;
//        p.g = 0;
//        p.b = 255;
//        q.x = cpds[a].second[0];
//        q.y = cpds[a].second[1];
//        q.z = cpds[a].second[2];
//        q.r = 255;
//        q.g = 0;
//        q.b = 0;
//		
//        int count = 0;
//        for(auto it = C2.begin(); it != C2.end(); ++it)
//        {
//            int b = it->second;
//            float dis1 = dis(cpds[a].first,cpds[b].first);
//            float dis2 = dis(cpds[a].second,cpds[b].second);
//            if(std::min(dis1/dis2, dis2/dis1) >= c0)
//                count++;
//        }
//        if(count >= 0.6*C2.size())
//        {
//            ++num;
//            C2.insert(temp);
//            view->addLine<pcl::PointXYZRGBA>(p,q,0,255,0, std::to_string(num));
//        }
//    }
//
//    view->addPointCloud<pcl::PointXYZRGBA> (cloud, "sample cloud");
//		while (!view->wasStopped ())
//		{
//			  view->spinOnce(100);
//		}
//	return 0;  
//}  
