/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor
  		Lidar* lidar1=new Lidar(cars, 0);
    // TODO:: Create point processor
  ProcessPointClouds<pcl::PointXYZ>* pointProcessor = new ProcessPointClouds<pcl::PointXYZ>();
  pcl::PointCloud<pcl::PointXYZ>::Ptr inputpoints;
  inputpoints=lidar1->scan();
  renderPointCloud(viewer,inputpoints,"pointcloud");
  std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr,pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor->SegmentPlane(inputpoints, 100, 0.2);
renderPointCloud(viewer,segmentCloud.first,"obstCloud",Color(1,0,0));
renderPointCloud(viewer,segmentCloud.second,"planeCloud",Color(0,1,0));
 
}
void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>& pointProcessor, pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud) {
    // Filtering
    auto filter_cloud = pointProcessor.FilterCloud(inputCloud, 0.2, Eigen::Vector4f (-20, -6, -5, 1), Eigen::Vector4f (30, 7, 5, 1));
  
    //Segmentation
    auto segment_cloud = pointProcessor.SegmentPlane(filter_cloud, 80, 0.3);
    // render ground plane with green
    renderPointCloud(viewer, segment_cloud.second, "planeCloud", Color(0, 1, 0));

    //Clustering
    auto cloud_clusters = pointProcessor.Clustering(segment_cloud.first, 0.42, 18, 1500);
    int cluster_ID = 0;
    std::vector<Color> colors = {Color(1, 0, 0), Color(1, 1, 0), Color(0, 0, 1)};

    for(const auto& cluster : cloud_clusters) {
        std::cout << "cluster size ";
        pointProcessor.numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(cluster_ID), colors[cluster_ID % colors.size()]);

        Box box = pointProcessor.BoundingBox(cluster);
        renderBox(viewer, box, cluster_ID);

        ++cluster_ID;
    }
}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);

    viewer->initCameraParameters();
    // distance in meters
    int distance = 15;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


int main (int argc, char** argv)
{
       std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = FPS;
    initCamera(setAngle, viewer);
    //simpleHighway(viewer);
    ProcessPointClouds<pcl::PointXYZI> pointProcessorI;
    std::vector<boost::filesystem::path> stream = pointProcessorI.streamPcd("../src/sensors/data/pcd/data_1");
    auto stream_iterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    while (!viewer->wasStopped()) {
        // remove old viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();
        // Load pcd data and process each frame
        inputCloudI = pointProcessorI.loadPcd((*stream_iterator).string());
        cityBlock(viewer, pointProcessorI, inputCloudI);

        stream_iterator++;
      //play till the end
        if(stream_iterator == stream.end())
            stream_iterator = stream.begin();

        // viewer spin
        viewer->spinOnce();}
}