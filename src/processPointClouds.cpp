// PCL lib Functions for processing point clouds

#include "processPointClouds.h"

//constructor:
template <typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}

//de-constructor:
template <typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}

template <typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    typename pcl::PointCloud<PointT>::Ptr filtercloud(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr Regioncloud(new pcl::PointCloud<PointT>());
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    //*************Voxel grid*************//

    pcl::VoxelGrid<PointT> vox;
    vox.setInputCloud(cloud);
    vox.setLeafSize(filterRes, filterRes, filterRes);
    vox.filter(*filtercloud);
    //***********CropBox****************//
    // pcl::CropBox<PointT> region(true);
    pcl::CropBox<PointT> region(true);
    region.setMax(maxPoint);
    region.setMin(minPoint);
    region.setInputCloud(filtercloud);
    region.filter(*Regioncloud);
    //*************Roof****************//

    pcl::CropBox<PointT> vehicleRoof(true);

    vehicleRoof.setMin(Eigen::Vector4f(-1.5, -1.7, -1.0, 1.0));
    vehicleRoof.setMax(Eigen::Vector4f(2.6, 1.7, -0.4, 1.0));
    vehicleRoof.setInputCloud(Regioncloud);

    std::vector<int> inlierIndices;
    vehicleRoof.filter(inlierIndices);

    for (int index : inlierIndices)
    {
        inliers->indices.push_back(index);
    }

    pcl::ExtractIndices<PointT> extractedIndices;

    extractedIndices.setInputCloud(Regioncloud);
    extractedIndices.setIndices(inliers);
    extractedIndices.setNegative(true);
    extractedIndices.filter(*Regioncloud);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return Regioncloud;
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud)
{
    // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane

    typename pcl::PointCloud<PointT>::Ptr obstcloud(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr planecloud(new pcl::PointCloud<PointT>());

    for (int index : inliers->indices)
    {
        planecloud->points.push_back(cloud->points[index]);
    }

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*obstcloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstcloud, planecloud);
    return segResult;
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    // TODO:: Fill in this function to find inliers for the cloud.
    pcl::SACSegmentation<PointT> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);

    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.size() == 0)
    {

        std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
        // PCL_ERROR("Could not estimate a planar model for the given dataset.");
        // return (-1);
    }
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers, cloud);
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    return segResult;
}

template <typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
    std::vector<pcl::PointIndices> cluster_indices;
    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);

    // euclidean cluster
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(clusterTolerance);
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    // get cluster cloud
    for (pcl::PointIndices get_indices : cluster_indices)
    {
        typename pcl::PointCloud<PointT>::Ptr cloud_cluster(new pcl::PointCloud<PointT>);

        for (int i : get_indices.indices)
            cloud_cluster->points.push_back(cloud->points[i]); 

        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size() << " data points." << std::endl;
        clusters.push_back(cloud_cluster);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}

template <typename PointT>
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

template <typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII(file, *cloud);
    std::cerr << "Saved " << cloud->points.size() << " data points to " + file << std::endl;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT>(file, *cloud) == -1) //* load the file
    {
        PCL_ERROR("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size() << " data points from " + file << std::endl;

    return cloud;
}

template <typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;
}

//**********************************************************//
template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr>
ProcessPointClouds<PointT>::RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations,
                                        const float distanceTol)
{

    std::unordered_set<int> inliersResult;

    // TODO: Fill in this function
    srand(time(NULL));

    while (maxIterations--)
    {

        std::unordered_set<int> inliers; // temporary map

        // insert three random indicies
        for (int i{0}; i < 3; i++)
        {
            inliers.insert(rand() % cloud->points.size());
        }

        auto iterator{std::begin(inliers)};

        const float x1{cloud->points.at((*iterator)).x};
        const float y1{cloud->points.at((*iterator)).y};
        const float z1{cloud->points.at((*iterator)).z};

        ++iterator;

        const float x2{cloud->points.at((*iterator)).x};
        const float y2{cloud->points.at((*iterator)).y};
        const float z2{cloud->points.at((*iterator)).z};

        // I have ABSOLUTELY no clue why the map was sometimes only inserting 2 indices
        // I shouldn't have to do this, but a seg fault was occurring without this check
        if (inliers.size() == 2)
        {
            inliers.insert(rand() % cloud->points.size());
        }

        ++iterator;

        const float x3{cloud->points.at((*iterator)).x};
        const float y3{cloud->points.at((*iterator)).y};
        const float z3{cloud->points.at((*iterator)).z};

        const float i{static_cast<float>(((y2 - y1) * (z3 - z1)) - ((z2 - z1) * (y3 - y1)))};
        const float j{static_cast<float>(((z2 - z1) * (x3 - x1)) - ((x2 - x1) * (z3 - z1)))};
        const float k{static_cast<float>(((x2 - x1) * (y3 - y1)) - ((y2 - y1) * (x3 - x1)))};

        // yeah, it's stupid, but it helps with following the given equations
        const float A{i}, B{j}, C{k};

        const float D{static_cast<float>(-((i * x1) + (j * y1) + (k * z1)))};

        const float euclideanDist{std::sqrt((A * A) + (B * B) + (C * C))};

        for (int index{0}; index < cloud->points.size(); index++)
        {
            const float plane{std::fabs((A * cloud->points.at(index).x) + (B * cloud->points.at(index).y) +
                                        (C * cloud->points.at(index).z) + D)};

            if ((plane / euclideanDist) <= distanceTol)
            {
                inliers.insert(index);
            }
        }

        if (inliers.size() > inliersResult.size())
        {
            inliersResult = inliers;
        }
    }

    typename pcl::PointCloud<PointT>::Ptr cloudInliers{new pcl::PointCloud<PointT>()};
    typename pcl::PointCloud<PointT>::Ptr cloudOutliers{new pcl::PointCloud<PointT>()};

    if (!inliersResult.empty())
    {
        for (int index{0}; index < cloud->points.size(); index++)
        {
            const PointT point{cloud->points.at(index)};

            if (inliersResult.count(index))
            {
                cloudInliers->points.push_back(point);
            }
            else
            {
                cloudOutliers->points.push_back(point);
            }
        }
    }

    return std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr>(
        cloudOutliers, cloudInliers);
}

template <typename PointT>
void ProcessPointClouds<PointT>::clusterHelper(int index, typename pcl::PointCloud<PointT>::Ptr cloud, std::vector<int>& cluster, std::vector<bool>& processed, KdTree* tree, float distanceTol) {
	processed[index] = true;
	cluster.push_back(index);
	std::vector<int> nearest = tree->search(cloud->points[index], distanceTol);

	for (int idx : nearest) {
		if (!processed[idx]) {
			clusterHelper(idx, cloud, cluster, processed, tree, distanceTol);
		}
	}
}
template <typename PointT>
// std::vector<std::vector<int>>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::euclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud, KdTree* tree, float distanceTol, int minSize, int maxSize)
{

	std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
	std::vector<bool> processed(cloud->points.size(), false);
	for(size_t idx = 0; idx < cloud->points.size(); ++idx)
	{
		if(processed[idx] == false)
		{
			std::vector<int> cluster_idx;
            typename pcl::PointCloud<PointT>::Ptr cloudCluster (new pcl::PointCloud<PointT>);
			clusterHelper(idx, cloud, cluster_idx, processed, tree, distanceTol);
            if(cluster_idx.size() >= minSize && cluster_idx.size() <= maxSize)
            {
                for(int i = 0; i < cluster_idx.size(); i++)
                {
                    PointT point;
                    point = cloud->points[cluster_idx[i]];
                    cloudCluster->points.push_back(point);
                }
                cloudCluster->width = cloudCluster->points.size();
                cloudCluster->height = 1;
                clusters.push_back(cloudCluster);
            }
            else{
                for(int i = 1; i < cluster_idx.size(); i++)
                {
                    processed[cluster_idx[i]] = false;
                }
            }
		}
	}
	return clusters;

}

