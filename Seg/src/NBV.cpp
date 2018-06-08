#include "NBV.h"

/*
TDF:known voxel radius ->5
getScore: num no influence
*/

NBV::NBV(const SegmentedPatch & nboPatch,
		 const std::vector<SegmentedPatch> & matchedModels,
		 const std::vector<double> & matchingErrors)
 : patch(nboPatch),
   modelNum(matchedModels.size()),
   models(matchedModels)
{
	computeConfidence(matchingErrors);

	for (int i = 0; i < modelNum; ++i)
	{
		std::cout << "confidence: " << confidence[i] << std::endl;
	}
}

std::queue<std::array<float, 6>> NBV::getNBVQueue(const Eigen::Matrix4f & camPose)
{
	Eigen::Vector3f lookAtPosition(0, 0, 0);
	Eigen::Vector3f radiusVector(0, 0, 0);
    
	for	(size_t i = 0; i < models.size(); ++i)
	{
		Eigen::Vector3f maxVec = patch.getCenterPosition();
		Eigen::Vector3f minVec = patch.getCenterPosition();

		maxVec[0] = std::max(maxVec[0], (float)models[i].getBoundingBox().xMax);
		maxVec[1] = std::max(maxVec[1], (float)models[i].getBoundingBox().yMax);
		maxVec[2] = std::max(maxVec[2], (float)models[i].getBoundingBox().zMax);
		minVec[0] = std::min(minVec[0], (float)models[i].getBoundingBox().xMin);
		minVec[1] = std::min(minVec[1], (float)models[i].getBoundingBox().yMin);
		minVec[2] = std::min(minVec[2], (float)models[i].getBoundingBox().zMin);

		lookAtPosition += confidence[i] * 0.5 * (maxVec + minVec);
		radiusVector += confidence[i] * 0.5 * (maxVec - minVec);
	}

	//get view points
	std::vector<Eigen::Vector3f> viewPositions(paraViewNum) ;

	constexpr float PI = 3.1415926;

	float dis = std::max(radiusVector[0], radiusVector[2]) + 0.8;
	for (int i = 0; i <  paraViewNum / 2; ++i)
	{
		Eigen::Vector3f shiftVector = {std::sin(4 * PI * i / paraViewNum) * dis, -0.2f, std::cos(4 * PI * i / paraViewNum) * dis};
		viewPositions[i] = lookAtPosition + shiftVector;
	}
	for (int i = paraViewNum / 2; i <  paraViewNum; ++i)
	{
		Eigen::Vector3f shiftVector = {std::sin(4 * PI * i / paraViewNum) * dis, -0.5f, std::cos(4 * PI * i / paraViewNum) * dis};
		viewPositions[i] = lookAtPosition + shiftVector;
	}

	auto pairComp =
		[](const std::pair<std::array<float, 6>, float> & p1, const std::pair<std::array<float, 6>, float> & p2)
		{ return p1.second < p2.second; };

	std::priority_queue<std::pair<std::array<float, 6>, float>,
		std::vector<std::pair<std::array<float, 6>, float>>,
		decltype(pairComp)> viewScorePriorityQueue(pairComp);

	for (int i = 0; i < paraViewNum; ++i)
	{
		double viewDistance = (camPose - computeNBVPose(lookAtPosition, viewPositions[i])).norm();
		std::cout << "View distance: " << viewDistance << std::endl;

		if (viewDistance < paraMinViewDistance)
			continue;

		float moveCost = (camPose.topRightCorner(3, 1) - viewPositions[i]).norm();

		float score =  computeViewScore(lookAtPosition, viewPositions[i]) + paraMoveCostWeight * moveCost;
		std::cout << "New score: " << score << std::endl;
		
		std::array<float, 6> eyeCenter;
		for (int j = 0; j < 3; ++j)
		{
			eyeCenter[j] = viewPositions[i][j];
			eyeCenter[j + 3] = lookAtPosition[j];
		}
		viewScorePriorityQueue.emplace(eyeCenter, score);
	}

	std::queue<std::array<float, 6>> nbvQueue;
	while (!viewScorePriorityQueue.empty())
	{
		nbvQueue.emplace(viewScorePriorityQueue.top().first);
		viewScorePriorityQueue.pop();
	}

	return nbvQueue;
}

float NBV::computeViewScore(const Eigen::Vector3f & lookAtPosition, const Eigen::Vector3f & viewPosition)
{
	std::vector<SegmentedPatch> viewModels;
	for (size_t i = 0; i < models.size(); ++i)
	{
		viewModels.emplace_back(models[i]);
	}
	SegmentedPatch viewPatch(patch);

	//trans pts
	for (size_t i = 0; i < viewModels.size(); ++i)
	{
		transformPointsToViewFrame(viewModels[i], lookAtPosition, viewPosition);
	}
	transformPointsToViewFrame(viewPatch, lookAtPosition, viewPosition);

	//voxelize
	float  voxel_size = 0.01;
	std::array<float, 3> voxelGridOrigin;
	std::array<int, 3> voxelGridDim;

	float * voxels = voxelize(viewModels, viewPatch, voxelGridOrigin, voxelGridDim);

	int voxelGridSize = voxelGridDim[0] * voxelGridDim[1] * voxelGridDim[2];

	//voxels in the view
	for (int i = 0; i < voxelGridDim[0]; ++i)
	{
		for (int j = 0; j < voxelGridDim[1]; ++j)
		{
			float midPt[3];
			midPt[0] = voxelGridOrigin[0] + i * voxel_size ;
			midPt[1] = voxelGridOrigin[1] + j * voxel_size;
			midPt[2] = voxelGridOrigin[2] + (voxelGridDim[2] + 10) * voxel_size;
			for (size_t k = 0; k < viewModels.size(); ++k)
			{
				rayTraceBlocks(&voxels[voxelGridSize * k], midPt, voxelGridOrigin, voxelGridDim, &voxels[voxelGridSize * viewModels.size()], confidence[k]);
			}
		}
	}

	float * voxel_TDF = new float[voxelGridSize * viewModels.size()];
	memset(voxel_TDF, 0, sizeof(float) * voxelGridSize *viewModels.size());

	computeTDFVoxelGridSun(voxels, voxel_TDF, voxelGridDim[0], voxelGridDim[1], voxelGridDim[2], paraVoxelSize, 0.05, models.size());

	float score = 0;
	for (size_t i = 0; i < viewModels.size(); ++i)
	{
		score += confidence[i] * getEntropy(i, confidence, voxel_TDF, voxelGridDim);
	}
	std::cout<<"viewscore"<<score<<std::endl;

	delete [] voxel_TDF;
	delete [] voxels;
	return score;
}

float NBV::getEntropy(int id, std::vector<double> confidence ,float* voxelTDF, const std::array<int, 3> & voxel_grid_dim)
{
    int voxel_grid_dim_x = voxel_grid_dim[0],voxel_grid_dim_y = voxel_grid_dim[1], voxel_grid_dim_z = voxel_grid_dim[2];
    int voxel_grid_size = voxel_grid_dim[0] * voxel_grid_dim[1] * voxel_grid_dim[2];
    int idx0 =  id * voxel_grid_size;

    std::vector<double> p = confidence;
    float entropy = 0;
    for (int ix = 0; ix < voxel_grid_dim_x; ++ix)
    {
        for (int iy = 0; iy < voxel_grid_dim_y; ++iy)
        {
              for (int iz = 0; iz < voxel_grid_dim_z; ++iz)
              {
                    std::vector<float> p1_;
                    int chose_idx = iz * voxel_grid_dim_x * voxel_grid_dim_y + iy * voxel_grid_dim_x + ix  + idx0;
                    float p1_i =  voxelTDF[chose_idx], p0_i;
                    float p0 = 0, p1 =0;
                    //pi on surface
                    if (p1_i > 0 && p1_i < 1)
                    {
                          p0_i = 1 - p1_i;
                    }
                    else if(p1_i < 0)
                    {
                        p1_i = 0;
                        p0_i = 1;
                    }
                    else
                    {
                        continue;
                    }
                  for (size_t j = 0; j < models.size(); j++)
                  {
                       int voxel_idx = iz * voxel_grid_dim_x * voxel_grid_dim_y + iy * voxel_grid_dim_x + ix  + j * voxel_grid_size;
                       float p1_j = voxelTDF[voxel_idx];
                       //pj on surface
                       if(p1_j > 0 && p1_j < 1)
                       {
                           p1_.push_back(p1_j);
                           p1 += p[j] * p1_j;
                           p0 += p[j] *(1-p1_j);
                       }
                       //pj not on surface
                       else
                       {
                            p1_.push_back(0);
                           // p1 += p[j] * 0;
                           p0 += p[j] * 1;
                       }
                  }
                  for (size_t j = 0; j < models.size(); j++)
                  {
                      float pj_1 = p[j] * p1_[j] / p1;
                      float pj_0=  p[j] * (1 - p1_[j]) / p0;
                      if( pj_1>0)
                        entropy  += p1_i * (-pj_1) * log(pj_1) ;
                      if(pj_0>0)
                        entropy  += p0_i * (-pj_0) * log(pj_0);
                  }
              }
         }
    }
     //std::cout<<"modelcore"<<score<<std::endl;
    return entropy;
}

void NBV::rayTraceBlocks(float *voxel,
						 float *midPt,
						 const std::array<float, 3> & voxel_grid_origin,
						 const std::array<int, 3> & voxel_grid_dim,
						 float *knownVoxel,
						 float coe_flag)
{
  	float voxel_size = 0.01;
	float startX,startY,startZ,endX,endY,endZ;
	startZ = voxel_grid_origin[2] / voxel_size;
	startX = midPt[0] * startZ / midPt[2] ;
	startY = midPt[1] * startZ / midPt[2] ;
	
	endZ =  midPt[2] / voxel_size ;
  	endX =  midPt[0] / voxel_size ;
	endY =  midPt[1] / voxel_size ;

	int gridX1  = floor(startX - voxel_grid_origin[0]/ voxel_size);
	int gridY1 = floor(startY - voxel_grid_origin[1]/ voxel_size);
	int gridZ1 = floor(startZ - voxel_grid_origin[2]/ voxel_size);
	
	int intX1 = floor(startX);
	int intY1 = floor(startY);
	int intZ1 = floor(startZ);

	int intX2 = floor(endX);
	int intY2 = floor(endY);
	int intZ2 = floor(endZ);
	
	if(gridX1 >= 0 && gridY1 >=0 && gridZ1 >= 0 && gridX1 < voxel_grid_dim[0] && gridY1 < voxel_grid_dim[1] && gridZ1 < voxel_grid_dim[2])
	{
		int grid = gridZ1 * voxel_grid_dim[1] * voxel_grid_dim[0] + gridY1 * voxel_grid_dim[0] + gridX1;
        if(voxel[grid] > 1 )
		{
            if(knownVoxel[grid] < 1)
				voxel[grid] = coe_flag;
			return;
		}
        else if(voxel[grid] > 0)
		{
			return;
		}

	}
	else
	{
		return;
	}
	while(intX1 != intX2 || intY1 != intY2 || intZ1 != intZ2)
	{
        if((startX== floor(startX)&& startY==floor(startY)) || (startX==floor(startX) && startZ==floor(startZ)) || (startY==floor(startY) && startZ==floor(startZ)))
        {
            return;
        }
		// 起点(当前)方块和终点方块XYZ不同(向某方向选了候选方块)  
    bool Xchanged = true;  
    bool Ychanged = true;  
    bool Zchanged = true;  
    // 各方向候选方块坐标  
    float newX;  
    float newY;  
    float newZ;  
    
     // 尝试向X方向选候选方块  
    if (intX2 > intX1)  
    {  
        newX = (float)intX1 + 1.0f;  
    }  
    else if (intX2 < intX1)  
    {  
        newX = (float)intX1;  
    }  
    else  
    {  
        Xchanged = false;  
    }  

    if (intY2 > intY1)  
    {  
        newY = (float)intY1 + 1.0f;  
    }  
    else if (intY2 < intY1)  
    {  
        newY = (float)intY1;  
    }  
    else  
    {  
        Ychanged = false;  
    }  

    if (intZ2 > intZ1)  
    {  
        newZ = (float)intZ1 + 1.0f;  
    }  
    else if (intZ2 < intZ1)  
    {  
        newZ = (float)intZ1;  
    }  
    else  
    {  
        Zchanged = false;  
    }  

    // 各方向候选方块离起点(当前)有多近，初始化为很大的数  
    float Xt = 9999999.0f;  
    float Yt = 9999999.0f;  
    float Zt = 9999999.0f;  
    float dX = endX - startX;  
    float dY = endY - startY;  
    float dZ = endZ - startZ;  

		// 向X方向选了候选方块  
    if (Xchanged)  
    {  
        Xt = (newX - startX) / dX;  
    }  

    if (Ychanged)  
    {  
        Yt = (newY - startY) / dY;  
    }  

    if (Zchanged)  
    {  
        Zt = (newZ - startZ) / dZ;  
    }  

    // 最终选了哪个方向的候选方块  
    int direction;  

    // 选出候选方块中离起点(当前)最近的，更新起点、要检测的方块坐标  
    if (Xt < Yt && Xt < Zt)  
    {  
        if (intX2 > intX1)  
        {  
            direction = 4;  
        }  
        else  
        {  
            direction = 5;  
        }  

        startX = newX;  
        startY += dY * Xt;  
        startZ += dZ * Xt;  
    }  
    else if (Yt < Zt)  
    {  
        if (intY2 > intY1)  
        {  
            direction = 0;  
        }  
        else  
        {  
            direction = 1;  
        }  

        startX += dX * Yt;  
        startY = newY;  
        startZ += dZ * Yt;  
    }  
    else  
    {  
        if (intZ2 > intZ1)  
        {  
            direction = 2;  
        }  
        else  
        {  
            direction = 3;  
        }  

        startX += dX * Zt;  
        startY += dY * Zt;  
        startZ = newZ;  
    }  

    intX1 = floor(startX);  
		gridX1  = floor(startX - voxel_grid_origin[0]/ voxel_size);
	
    if (direction == 5) // X-方向  
    {  
        // MC以方块内各轴最小坐标为方块坐标，这里得到的是X上最大坐标所以要-1  
        --intX1;  
        --gridX1;
    }  

    intY1 = floor(startY);  
		gridY1 = floor(startY - voxel_grid_origin[1]/ voxel_size);
	  
    if (direction == 1) // Y-方向  
    {  
        --intY1;  
        --gridY1;
    }  
		gridZ1 = floor(startZ - voxel_grid_origin[2]/ voxel_size);
    intZ1 = floor(startZ);  

    if (direction == 3) // Z-方向  
    {  
        --intZ1;
        --gridZ1;  
    }  
    
    //std::cout<<"3"<<intX1<<" "<<intY1<<" "<<intZ1<<std::endl;
    if(gridX1 >= 0 && gridY1 >=0 && gridZ1 >= 0 && gridX1 < voxel_grid_dim[0] && gridY1 < voxel_grid_dim[1] && gridZ1 < voxel_grid_dim[2])
		{
		  int grid = gridZ1 * voxel_grid_dim[1] * voxel_grid_dim[0] + gridY1 * voxel_grid_dim[0] + gridX1;
		  
			if(voxel[grid] > 1)
			{	
                if(knownVoxel[grid] < 1)
					voxel[grid] = coe_flag;				
				return;
			}
            else if(voxel[grid] > 0)
			{
				return;
			}
            voxel[grid] = -1;
		}
		else
		{

			return;
		}
	}
}

float * NBV::voxelize(const std::vector<SegmentedPatch> & models,
					  const SegmentedPatch & patch,
					  std::array<float, 3> & voxelGridOrigin,
					  std::array<int, 3> & voxelGridDim)
{
    //pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);

    // Compute point cloud coordinates of the origin voxel (0,0,0) of the voxel grid
	std::cout << "Model size: " << models.size() << std::endl;
	std::array<float, 3> voxelGridMax;

    voxelGridOrigin[0] = std::numeric_limits<float>::max();
    voxelGridOrigin[1] = std::numeric_limits<float>::max();
    voxelGridOrigin[2] = std::numeric_limits<float>::max();
	voxelGridMax[0] = -std::numeric_limits<float>::max();
	voxelGridMax[1] = -std::numeric_limits<float>::max();
	voxelGridMax[2] = -std::numeric_limits<float>::max();

    for (int i = 0; i < patch.getPointNum(); ++i)
    {
		voxelGridOrigin[0] = std::min(voxelGridOrigin[0], patch.getPointPositions()[i][0]);
		voxelGridOrigin[1] = std::min(voxelGridOrigin[1], patch.getPointPositions()[i][1]);
		voxelGridOrigin[2] = std::min(voxelGridOrigin[2], patch.getPointPositions()[i][2]);
		voxelGridMax[0] = std::max(voxelGridMax[0], patch.getPointPositions()[i][0]);
		voxelGridMax[1] = std::max(voxelGridMax[1], patch.getPointPositions()[i][1]);
		voxelGridMax[2] = std::max(voxelGridMax[2], patch.getPointPositions()[i][2]);
    }

    for (size_t i = 0; i < models.size(); ++i)
    {
        for (int j = 0; j < models[i].getPointNum(); ++j)
        {
            voxelGridOrigin[0] = std::min(voxelGridOrigin[0], models[i].getPointPositions()[j][0]);
            voxelGridOrigin[1] = std::min(voxelGridOrigin[1], models[i].getPointPositions()[j][1]);
            voxelGridOrigin[2] = std::min(voxelGridOrigin[2], models[i].getPointPositions()[j][2]);
            voxelGridMax[0] = std::max(voxelGridMax[0], models[i].getPointPositions()[j][0]);
            voxelGridMax[1] = std::max(voxelGridMax[1], models[i].getPointPositions()[j][1]);
            voxelGridMax[2] = std::max(voxelGridMax[2], models[i].getPointPositions()[j][2]);
        }
    }

    voxelGridDim[0] = floor((voxelGridMax[0] - voxelGridOrigin[0]) / paraVoxelSize) + 1 + paraVoxelGridPadding * 2;
    voxelGridDim[1] = floor((voxelGridMax[1] - voxelGridOrigin[1]) / paraVoxelSize) + 1 + paraVoxelGridPadding * 2;
    voxelGridDim[2] = floor((voxelGridMax[2] - voxelGridOrigin[2]) / paraVoxelSize) + 1 + paraVoxelGridPadding * 2;

    voxelGridOrigin[0] = voxelGridOrigin[0] - paraVoxelGridPadding * paraVoxelSize + paraVoxelSize / 2;
    voxelGridOrigin[1] = voxelGridOrigin[1] - paraVoxelGridPadding * paraVoxelSize + paraVoxelSize / 2;
    voxelGridOrigin[2] = voxelGridOrigin[2] - paraVoxelGridPadding * paraVoxelSize + paraVoxelSize / 2;

    std::cout << "Size of voxel grid: " << voxelGridDim[0] << " x " << voxelGridDim[1] << " x " << voxelGridDim[2] << std::endl;

    // Compute surface occupancy grid
    int voxelGridSize = voxelGridDim[0] * voxelGridDim[1] * voxelGridDim[2];
    float * voxelGridOcc = new float[voxelGridSize * (models.size() + 1)];
    memset(voxelGridOcc, 0, sizeof(float) * voxelGridSize * (models.size() + 1));

    for (size_t i = 0; i < models.size(); ++i)
    {
        for(int j = 0; j < models[i].getPointNum(); ++j)
        {
            int pointGridX = floor((models[i].getPointPositions()[j][0] - voxelGridOrigin[0]) / paraVoxelSize);
            int pointGridY = floor((models[i].getPointPositions()[j][1] - voxelGridOrigin[1]) / paraVoxelSize);
            int pointGridZ = floor((models[i].getPointPositions()[j][2] - voxelGridOrigin[2]) / paraVoxelSize);

            voxelGridOcc[voxelGridSize * i + pointGridZ * voxelGridDim[1] * voxelGridDim[0] + pointGridY * voxelGridDim[0] + pointGridX] = confidence[i] + 1;

            //pcl::PointXYZRGBA p;
            //p.x = pointGridX * paraVoxelSize;//+ v_origin[0];
            //p.y = pointGridY * paraVoxelSize;//+ v_origin[1];
            //p.z = pointGridZ * paraVoxelSize;//+ v_origin[2];
            //p.r = 255 - 255 / models.size() * i;
            //p.g = 255 / models.size() * i;
            //p.b = 255 / models.size() * i;
            //cloud->points.push_back(p);
         }
    }

    for(int i = 0; i < patch.getPointNum(); ++i)
    {
        int pointGridX = std::floor((patch.getPointPositions()[i][0] - voxelGridOrigin[0]) / paraVoxelSize);
        int pointGridY = std::floor((patch.getPointPositions()[i][1] - voxelGridOrigin[1]) / paraVoxelSize);
        int pointGridZ = std::floor((patch.getPointPositions()[i][2] - voxelGridOrigin[2]) / paraVoxelSize);
        voxelGridOcc[voxelGridSize * models.size() + pointGridZ * voxelGridDim[1] * voxelGridDim[0] + pointGridY * voxelGridDim[0] + pointGridX] = 2; 
        //pcl::PointXYZRGBA p;
        //p.x = pointGridX * paraVoxelSize;//+ v_origin[0];
        //p.y = pointGridY * paraVoxelSize;//+ v_origin[1];
        //p.z = pointGridZ * paraVoxelSize;//+ v_origin[2];
        //p.r = 255;
        //p.g = 255;
        //p.b = 255;
        //cloud->points.push_back(p);      
	}
	//pcl::io::savePLYFileASCII("abc.ply", *cloud);
    //pcl::visualization::CloudViewer viewer ("Viewer");
   //viewer.showCloud (cloud);
   //while (!viewer.wasStopped ())
    //{
    //}
    return voxelGridOcc;

 }

void NBV::transformPointsToViewFrame(SegmentedPatch & pointCloud, const Eigen::Vector3f & lookAtPosition, const Eigen::Vector3f & viewPosition)
{
	Eigen::Vector3f zAxis = (lookAtPosition - viewPosition).normalized();
	Eigen::Vector3f upVector(0, -1, 0);

	Eigen::Vector3f xAxis = (upVector.cross(zAxis)).normalized();
	Eigen::Vector3f yAxis = (zAxis.cross(xAxis)).normalized();

	Eigen::Matrix3f rotMat;
	rotMat.row(0) = xAxis.transpose();
	rotMat.row(1) = yAxis.transpose();
	rotMat.row(2) = zAxis.transpose();

	Eigen::Vector3f transVec;
	transVec[0] = -xAxis.dot(viewPosition);
	transVec[1] = -yAxis.dot(viewPosition);
	transVec[2] = -zAxis.dot(viewPosition);

	Eigen::Matrix4f viewTransMat;
	viewTransMat.setIdentity();
	viewTransMat.topLeftCorner(3, 3) = rotMat;
	viewTransMat.topRightCorner(3, 1) = transVec;

	pointCloud *= viewTransMat;

    //pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);	
	//
    ////view基pts
    //float pt[3];
    //for (int pt_idx = 0; pt_idx < pts[0]; ++pt_idx)
    //{
    //    pt[0] = pts[pt_idx * 3 + 1];  pt[1] = pts[pt_idx * 3 + 2];  pt[2] = pts[pt_idx * 3 + 3];
    //    MatrixmulVec(pt,T,pt);
    //    pts[pt_idx * 3 + 1] = pt[0];  pts[pt_idx * 3 + 2] = pt[1];  pts[pt_idx * 3 + 3] = pt[2];

	//pcl::PointXYZRGBA p;
    //    p.x = pts[3 * pt_idx + 1];
    //    p.y = pts[3 * pt_idx + 2];
    //    p.z = pts[3 * pt_idx + 3];
    //    p.r = 255;
    //    p.g = 255;
    //    p.b = 255;
    //    cloud->points.push_back(p);   
    //} 

    //pcl::io::savePLYFileASCII("test.ply",*cloud);
}

void NBV::computeConfidence(const std::vector<double> & matchingErrors)
{
	confidence.clear();

	if (matchingErrors.size() == 1)
	{
		confidence.push_back(1.0);
		return;
	}

	double confidenceSum = 0;
	for (auto error : matchingErrors)
	{
		//confidence.push_back(exp(-pow(error - paraGaussFunctionCenter, 2) / 0.001));
		confidence.push_back(exp(-pow(error - paraGaussFunctionCenter, 2) / (2 * pow(paraGaussFunctionWidth, 2))));
		confidenceSum += confidence.back();
	}

	for (auto & conf : confidence)
		conf /= confidenceSum;
}

Eigen::Matrix4f NBV::computeNBVPose(const Eigen::Vector3f & lookAtPosition, const Eigen::Vector3f & viewPosition)
{
	Eigen::Vector3f upVector(0, -1, 0);
	Eigen::Matrix3f nbvRotMat;

	Eigen::Vector3f nbvZ = (lookAtPosition - viewPosition).normalized();
	Eigen::Vector3f nbvX = nbvZ.cross(upVector).normalized();
	Eigen::Vector3f nbvY = nbvZ.cross(nbvX).normalized();

	nbvRotMat.col(0) = nbvX;
	nbvRotMat.col(1) = nbvY;
	nbvRotMat.col(2) = nbvZ;

	Eigen::Matrix4f nbvPose;
	nbvPose.setIdentity();
	nbvPose.topLeftCorner(3, 3) = nbvRotMat;
	nbvPose.topRightCorner(3, 1) = viewPosition;
	return nbvPose;
}
