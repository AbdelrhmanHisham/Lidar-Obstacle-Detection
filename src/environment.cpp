/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"
// #include "quiz/cluster/kdtree.h"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr &viewer)
{

    Car egoCar(Vect3(0, 0, 0), Vect3(4, 2, 2), Color(0, 1, 0), "egoCar");
    Car car1(Vect3(15, 0, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car1");
    Car car2(Vect3(8, -4, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car2");
    Car car3(Vect3(-12, 4, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car3");
    Car car4(Vect3(-5, 10, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car4");

    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);
    cars.push_back(car4);

    if (renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
        car4.render(viewer);
    }

    return cars;
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr &viewer,
               ProcessPointClouds<pcl::PointXYZI> *pointProcessorI,
               const pcl::PointCloud<pcl::PointXYZI>::Ptr &inputCloud)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display City Block     -----
    // ----------------------------------------------------

//     constexpr float X{30.0}, Y{6.5}, Z{2.5};

//     const pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud{pointProcessorI->FilterCloud(
//         inputCloud, 0.1f, Eigen::Vector4f(-(X / 2), -Y, -Z, 1), Eigen::Vector4f(X, Y, Z, 1))};

//     renderPointCloud(viewer, filteredCloud, "filteredCloud");

//     // std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->SegmentPlane(filteredCloud, 20, 0.2);
//     std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->RansacPlane(filteredCloud, 20, 0.2);
//      renderPointCloud(viewer,segmentCloud.first,"obstCloud",Color(0,0,1));
//      renderPointCloud(viewer,segmentCloud.second,"planeCloud",Color(0,1,0));


//     // std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters{pointProcessorI->Clustering(
//     //     segmentCloud.first, 1.0, 3, 30)};

//     int clusterId{0};
//     std::vector<Color> colors{Color(1, 0, 0), Color(0, 1, 0), Color(0, 0, 1)};
// KdTree* tree = new KdTree;
// for(int i =0 ; i < segmentCloud.first->points.size() ; i++){

//     tree->insert(segmentCloud.first->points[i],i);
// } 
//   std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->euclideanCluster(segmentCloud.first, tree, 0.5, 30, 250);
//     bool renderBoundingBox{true};

//     for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
//     {
//         std::cout << "cluster size ";
//         pointProcessorI->numPoints(cluster);
//         renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors.at(clusterId));

//         if (renderBoundingBox)
//         {
//             Box box(pointProcessorI->BoundingBox(cluster));
//             renderBox(viewer, box, clusterId);
//         }

//         ++clusterId;
//     }


/**************************************/
 pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = pointProcessorI->FilterCloud(inputCloud, 0.3 , Eigen::Vector4f (-20, -6, -3, 1), Eigen::Vector4f ( 30, 7, 2, 1));
  //renderPointCloud(viewer,filterCloud,"filterCloud");
  std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->RansacPlane(filterCloud, 100, 0.2);
  KdTree *tree = new KdTree;

  for (int i=0; i<segmentCloud.first->points.size(); i++) 
    tree->insert(segmentCloud.first->points[i],i);

  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->euclideanCluster(segmentCloud.first, tree, 0.7, 7, 100);

  renderPointCloud(viewer,segmentCloud.second, "planefield", Color(0,1,1));
  renderPointCloud(viewer,segmentCloud.first, "obsfield", Color(1,1,0));
  int clusterId = 0;
  int colorIndex = 0;
  std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};

  for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
  {
    std::cout << "cluster size ";
    pointProcessorI->numPoints(cluster);
    renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors.at(colorIndex));
    
    Box box = pointProcessorI->BoundingBox(cluster);
    renderBox(viewer,box,clusterId);
    ++clusterId;
    ++colorIndex;
    colorIndex %= colors.size();
  }
/**************************************/



}

void simpleHighway(pcl::visualization::PCLVisualizer::Ptr &viewer)
{

    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);

    // TODO:: Create lidar sensor
    Lidar *lidar = new Lidar(cars, 0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputcloud = lidar->scan();
    // renderRays(viewer,lidar->position,inputcloud);
    // renderPointCloud(viewer,inputcloud,"input");

    // TODO:: Create point processor
    // ProcessPointClouds<pcl::PointXYZ> * pointproccesor = new ProcessPointClouds<pcl::PointXYZ>();
    ProcessPointClouds<pcl::PointXYZ> pointproccesor;

    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointproccesor.SegmentPlane(inputcloud, 100, 0.2);
    //  std::pair<typename pcl::PointCloud<pcl::PointXYZ>::Ptr, typename pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointproccesor.SegmentPlane(inputcloud, 100, 0.2);
    //  renderPointCloud(viewer,segmentCloud.first,"obstCloud",Color(0,0,1));
    //  renderPointCloud(viewer,segmentCloud.second,"planeCloud",Color(0,1,0));
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointproccesor.Clustering(segmentCloud.first, 1.0, 3, 30);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1, 0, 0), Color(0, 1, 0), Color(0, 0, 1), Color(1, 1, 1)};

    for (pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
        std::cout << "cluster size ";
        pointproccesor.numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors[clusterId]);
        ++clusterId;
    }
}

//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr &viewer)
{

    viewer->setBackgroundColor(0, 0, 0);

    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;

    switch (setAngle)
    {
    case XY:
        viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0);
        break;
    case TopDown:
        viewer->setCameraPosition(0, 0, distance, 1, 0, 1);
        break;
    case Side:
        viewer->setCameraPosition(0, -distance, 0, 0, 0, 1);
        break;
    case FPS:
        viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if (setAngle != FPS)
        viewer->addCoordinateSystem(1.0);
}

int main(int argc, char **argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);

    // simpleHighway(viewer);
    // cityBlock(viewer);

    ProcessPointClouds<pcl::PointXYZI> *pointProcessorI{new ProcessPointClouds<pcl::PointXYZI>()};

    std::vector<boost::filesystem::path> stream{pointProcessorI->streamPcd(
        "../src/sensors/data/pcd/data_1")};

    auto streamIterator{stream.begin()};

    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;
    while (!viewer->wasStopped())
    {

        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
        cityBlock(viewer, pointProcessorI, inputCloudI);
    //  renderPointCloud(viewer,inputCloudI,"INPUT");

        streamIterator++;

        if (streamIterator == stream.end()) { streamIterator = stream.begin(); }

        viewer->spinOnce();
           }
}