#include <chrono>
#include <thread>
#include <iostream>

#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "Registration/icp.h"

void colorize(const pcl::PointCloud<pcl::PointXYZ>& cloud, pcl::PointCloud<pcl::PointXYZRGB> & cloud_colored, const std::vector<int> &color)
{
    cloud_colored.clear();
    pcl::PointXYZRGB pt_tmp;
    for(int i = 0; i < cloud.size(); ++i)
    {
        const auto& pt = cloud.points[i];
        pt_tmp.x = pt.x;
        pt_tmp.y = pt.y;
        pt_tmp.z = pt.z;
        pt_tmp.r = color[0];
        pt_tmp.g = color[1];
        pt_tmp.b = color[2];
        cloud_colored.points.emplace_back(pt_tmp);
    }
}

int main(int argc, char **argv)
{
    using namespace std;
    using namespace std::chrono_literals;

    using PointT = pcl::PointXYZ;
    using PointCloudT = pcl::PointCloud<PointT>;
    using PointCloudPtr = pcl::PointCloud<PointT>::Ptr;

    std::string source_path, target_path;

    if (argc == 1)
    {
        source_path = SOURCE_CLOUD_PATH;
        target_path = TARGET_CLOUD_PATH;
    }
    else if(argc == 3)
    {
        source_path = argv[1];
        target_path = argv[2];
    }
    else
    {
        std::cerr << "Usage: " << argv[0] << " [source.pcd] [target.pcd]" << std::endl;
        return 1;
    }

    // load pcd files
    auto source_cloud = pcl::make_shared<PointCloudT>();
    auto target_cloud = pcl::make_shared<PointCloudT>();
    if(pcl::io::loadPCDFile<pcl::PointXYZ>(source_path, *source_cloud) == -1)
    {
        PCL_ERROR("Couldn't read pcd file \n");
        return -1;
    }

    if(pcl::io::loadPCDFile<pcl::PointXYZ>(target_path, *target_cloud) == -1)
    {
        PCL_ERROR("Couldn't read pcd file \n");
        return -1;
    }

    // voxelization
    auto source_voxelized = pcl::make_shared<PointCloudT>();
    auto target_voxelized = pcl::make_shared<PointCloudT>();

    pcl::VoxelGrid<PointT> voxel_filter;
    voxel_filter.setLeafSize(0.2f, 0.2f, 0.2f);
    
    voxel_filter.setInputCloud(source_cloud);
    voxel_filter.filter(*source_voxelized);
    voxel_filter.setInputCloud(target_cloud);
    voxel_filter.filter(*target_voxelized);


    auto aligned_cloud = pcl::make_shared<PointCloudT>();

    ICP<PointT> icp;
    icp.setInputSource(source_voxelized);
    icp.setInputTarget(target_voxelized);

    auto t0 = std::chrono::system_clock::now();
    icp.align(*aligned_cloud);
    std::cout << "ICP elapsed time : " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - t0).count() << " ms\n";

    if(icp.hasConverged())
    {
        std::cout << "ICP converged!" << std::endl;
    }
    else
    {
        std::cout << "ICP not converged!" << std::endl;
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_colored(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr aligned_colored(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr target_colored(new pcl::PointCloud<pcl::PointXYZRGB>);

    colorize(*source_voxelized, *source_colored, {255, 0, 0});
    colorize(*aligned_cloud, *aligned_colored, {0, 255, 0});
    colorize(*target_voxelized, *target_colored, {0, 0, 255});


    pcl::visualization::CloudViewer cloud_viewer("Cloud Viewer");
    cloud_viewer.showCloud(source_colored, "source");
    cloud_viewer.showCloud(aligned_colored, "align");
    cloud_viewer.showCloud(target_colored, "target");
    while(!cloud_viewer.wasStopped())
    {
        std::this_thread::sleep_for(30ms);
    }

    return 0;
}