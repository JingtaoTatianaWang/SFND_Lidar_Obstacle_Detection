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
    /*** Voxel grid point reduction and region based filtering ***/

    typename pcl::PointCloud<PointT>::Ptr filtered_cloud(new pcl::PointCloud<PointT>());
    // down-sample the dataset
    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(filterRes, filterRes, filterRes);
    vg.filter(*filtered_cloud);

    // Filter to keep the points in range
    pcl::CropBox<PointT> region(true);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(filtered_cloud);
    region.filter(*filtered_cloud);

    std::vector<int> indices;
    // Filter point cloud on the roof of host vehicle
    region.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
    region.setMax(Eigen::Vector4f(2.6, 1.7, -0.4, 1));
    region.setInputCloud(filtered_cloud);
    region.filter(indices);

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    for (int index : indices) {
        inliers->indices.push_back(index);
    }

    // Create the filtering object
    pcl::ExtractIndices<PointT> extract;
    // Extract the point cloud on roof
    extract.setInputCloud(filtered_cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*filtered_cloud);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return filtered_cloud;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: According to the index of plane (inliers), Create two new point clouds, one cloud with obstacles and other with segmented plane

    typename pcl::PointCloud<PointT>::Ptr obstacle_cloud(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr plane_cloud(new pcl::PointCloud<PointT>());

    // Copy inliers point cloud as plane
    for (int index : inliers->indices) {
        plane_cloud->points.push_back(cloud->points[index]);
    }

    // Create the filtering obstacles
    pcl::ExtractIndices<PointT> extract;
    // Extract the inliers so that we can get obstacles point cloud
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*obstacle_cloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstacle_cloud, plane_cloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    // TODO:: Fill in this function to find inliers for the cloud.
    std::unordered_set<int> inliersResult;
    // To obtain a seed for the random number engine
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0 , cloud->points.size());
    // For max iterations 
  float x[3], y[3], z[3],a,b,c,d,distance;

    while (maxIterations--) {
        // Randomly sample subset: pick three points
        std::unordered_set<int> inliers;
        // avoid picking the same point
      while (inliers.size() < 3) {
        inliers.insert(dis(gen));
      }
      auto itr = inliers.begin();
      const int ind0 = *itr++;
      const int ind1 = *itr++;
      const int ind2 = *itr;
      

        float v0[3]= {cloud->points[ind1].x - cloud->points[ind0].x, cloud->points[ind1].y - cloud->points[ind0].y,cloud-> points[ind1].z - cloud->points[ind0].z};
        float v1[3] = {cloud->points[ind2].x - cloud->points[ind0].x, cloud->points[ind2].y - cloud->points[ind0].y, cloud->points[ind2].z - cloud->points[ind0].z};

         a = v0[1] * v1[2] - v0[2] * v1[1];
         b = v0[2] * v1[0] - v0[0] * v1[2];
         c = v0[0] * v1[1] - v0[1] * v1[0];
         d = -(a * cloud->points[ind0].x + b * cloud->points[ind0].y+ c * cloud->points[ind0].z);


        // Measure distance between every point and fitted line
        for (int index = 0; index < cloud->points.size(); index++) {
            if (inliers.count(index) > 0) {
                continue;
            }

            auto point = cloud->points[index];

            distance = fabs(a* point.x + b* point.y + c* point.z+d) / sqrt(a*a + b*b + c*c);
            // If distance is smaller than threshold count it as inlier
            if (distance <= distanceThreshold) {
                inliers.insert(index);
            }
        }

        // Return indices of inliers from fitted plane with most inliers
        if (inliers.size() > inliersResult.size()) {
            inliersResult = inliers;
        }
    }

    pcl::PointIndices::Ptr inliers_result(new pcl::PointIndices());
    for (auto i : inliersResult) {
        inliers_result->indices.push_back(i);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers_result,cloud);
    return segResult;
}

template<typename PointT>
void ProcessPointClouds<PointT>::clusterHelper(int indice, const std::vector<std::vector<float>>& points, std::vector<int>& cluster, std::vector<bool>& processed, KdTree* tree, float distanceTol) {
    processed[indice] = true;
    cluster.push_back(indice);

    std::vector<int> nearest = tree->search(points[indice], distanceTol);

    for (int id : nearest) {
        if (!processed[id]) {
            clusterHelper(id, points, cluster, processed, tree, distanceTol);
        }
    }
}

template<typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol) {

    std::vector<std::vector<int>> clusters;

    std::vector<bool> processed(points.size(), false);

    int i = 0;
    while (i < points.size()) {
        if (processed[i]) {
            i++;
            continue;
        }

        std::vector<int> cluster;
        clusterHelper(i, points, cluster, processed, tree, distanceTol);
        clusters.push_back(cluster);
        i++;
    }


    // Return list of indices for each cluster
    return clusters;

}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    auto* tree = new KdTree;

    // build kdtree
    std::vector<std::vector<float>> points;
    for (int i=0; i< cloud->points.size(); i++) {
        std::vector<float> point({cloud->points[i].x, cloud->points[i].y, cloud->points[i].z});
        points.push_back(point);
        tree->insert(points[i], i);
    }

    std::vector<std::vector<int>> cluster_indices = euclideanCluster(points, tree, clusterTolerance);

    for (const auto& get_indices : cluster_indices) {
        typename pcl::PointCloud<PointT>::Ptr cloud_cluster(new pcl::PointCloud<PointT>());

        for (const auto index : get_indices) {
            cloud_cluster->points.push_back(cloud->points[index]);
        }

        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        if (cloud_cluster->width >= minSize && cloud_cluster->width <= maxSize) {
            clusters.push_back(cloud_cluster);
        }
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
    // sort files inoder to play in time s
    sort(paths.begin(), paths.end());

    return paths;

}