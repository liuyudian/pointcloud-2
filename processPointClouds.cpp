// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"

//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    pcl::VoxelGrid<PointT> vg;
    typename pcl::PointCloud<PointT>::Ptr cloudFiltered(new pcl::PointCloud<PointT>);

    vg.setInputCloud(cloud);
    vg.setLeafSize(filterRes,filterRes,filterRes);
    vg.filter(*cloudFiltered);

    typename pcl::PointCloud<PointT>::Ptr cloudRegion(new pcl::PointCloud<PointT>);

    pcl::CropBox<PointT> region(true);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(cloudFiltered);
    region.filter(*cloudRegion);

    std::vector<int> indices;

    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f(-1.5,-1.7,-1,1));
    roof.setMax(Eigen::Vector4f(2.6,1.7,-0.4,1));
    roof.setInputCloud(cloudRegion);
    roof.filter(indices);

    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    for(int point:indices)
        inliers->indices.push_back(point);
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloudRegion);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloudRegion);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloudRegion;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr obstCloud (new pcl::PointCloud<PointT> ());
    typename pcl::PointCloud<PointT>::Ptr planeCloud (new pcl::PointCloud<PointT> ());

    for(int index : inliers -> indices)
        planeCloud->points.push_back(cloud->points[index]);

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*obstCloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    // TODO:: Fill in this function to find inliers for the cloud.

    std::unordered_set<int> inlierResult;
    for(int i=0;i<maxIterations;++i)
	{
		// Randomly sample subset and fit line
		int first;
		int second;
		int third;
		std::unordered_set<int> inlier;
		do
		{
			first = rand()%(cloud->size());
			second = rand()%(cloud->size());
			third = rand()%(cloud->size());
		}while(first==second || second == third || third == first);
		std::vector<float> v1,v2;
		v1.push_back(cloud->points[second].x - cloud->points[first].x);
		v1.push_back(cloud->points[second].y - cloud->points[first].y);
		v1.push_back(cloud->points[second].z - cloud->points[first].z);

		v2.push_back(cloud->points[third].x - cloud->points[first].x);
		v2.push_back(cloud->points[third].y - cloud->points[first].y);
		v2.push_back(cloud->points[third].z - cloud->points[first].z);


		auto a = v1[1]*v2[2] - v1[2]*v2[1];
		auto b = v1[2]*v2[0] - v1[0]*v2[2];
		auto c = v1[0]*v2[1] - v1[1]*v2[0];
		auto d = -(a*cloud->points[first].x+b*cloud->points[first].y+c*cloud->points[first].z);
		// Measure distance between every point and fitted line
		for(int j=0;j<cloud->size();j++)
		{
			PointT point = cloud->points[j];
			auto distance = fabs(a*point.x+b*point.y+c*point.z+d)/(sqrt(a*a+b*b+c*c));
			if(distance<=distanceThreshold)
			{
				inlier.insert(j);
			}
		}
		// If distance is smaller than threshold count it as inlier
		if(inlier.size()>inlierResult.size())
		    inlierResult = inlier;
	}

    pcl::PointIndices::Ptr inliers (new pcl::PointIndices());
    for(int tmp : inlierResult)
        inliers->indices.push_back(tmp);
    if(inliers->indices.size()==0)
    {
        std::cout<<"could not estimate a planar model for the given dataset."<<std::endl;
    }

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;
    return segResult;
}
template<typename PointT>
void ProcessPointClouds<PointT>::clusterHelper(std::vector<std::vector<float> > treeInput,std::vector<bool>& flags,pcl::PointIndices& cluster,int point,KdTree* tree,float distanceTol)
{
	flags[point]=true;
	cluster.indices.push_back(point);
	std::vector<int> nearbys = tree->search(treeInput[point],distanceTol);
	for(int index =0;index<nearbys.size();index++)
		if(!flags[nearbys[index]])
			clusterHelper(treeInput,flags,cluster,nearbys[index],tree,distanceTol);
}
template<typename PointT>
std::vector<pcl::PointIndices> ProcessPointClouds<PointT>::euclideanCluster(std::vector<std::vector<float> > treeInput, KdTree* tree, float distanceTol)
{

	// TODO: Fill out this function to return list of indices for each cluster

	std::vector<pcl::PointIndices> clusters;
	std::vector<bool> flags;
	for(int index=0;index<treeInput.size();index++)
	{
		flags.push_back(false);
		//tree->insert(points[index],index);
	}
	for(int index=0;index<treeInput.size();index++)
	{
		if(flags[index])
			continue;
		pcl::PointIndices cluster;
		clusterHelper(treeInput,flags,cluster,index,tree,distanceTol);
		clusters.push_back(cluster);
	}
	return clusters;
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles

    KdTree* tree = new KdTree;

    std::vector<std::vector<float> > treeInput;
    for (int i=0; i<cloud->points.size(); i++) 
    {
        std::vector<float> p;
        p.push_back(cloud->points[i].x);
        p.push_back(cloud->points[i].y);
        p.push_back(cloud->points[i].z);
        tree->insert(p,i);
        treeInput.push_back(p);
    } 

    std::vector<pcl::PointIndices> clusterIndices = euclideanCluster(treeInput,tree,clusterTolerance);

    for(pcl::PointIndices getIndices:clusterIndices)
    {
        typename pcl::PointCloud<PointT>::Ptr cloudCluster (new pcl::PointCloud<PointT>);

        for(int index : getIndices.indices)
            cloudCluster->points.push_back(cloud->points[index]);
        
        cloudCluster->width = cloudCluster->points.size();
        cloudCluster->height = 1;
        cloudCluster->is_dense = true;

        clusters.push_back(cloudCluster);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}