#include <chrono>
#include <thread>
#include <iostream>

#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/registration/icp.h>
#include "Registration/kdtree.h"
#include "Registration/timer.h"

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

    pcl::KdTreeFLANN<PointT> pcl_tree;
    pcl_tree.setInputCloud(source_voxelized);

    KDTree tree;
    tree.setInputCloud(source_voxelized);

    float dist_tree;
    int idx_tree;
    std::vector<int> idx_pcl_tree(1);
    std::vector<float> dist_pcl_tree(1);

    for(int i = 0; i < 10; ++i)
    {
        tree.nearestSearch(target_voxelized->points[i], dist_tree, idx_tree);
        pcl_tree.nearestKSearch(target_voxelized->points[i], 1, idx_pcl_tree, dist_pcl_tree);

        std::cout << "tree idx, dist = " << idx_tree << ", " << dist_tree << "\n";
        std::cout << "pcl  idx, dist = " << idx_pcl_tree[0] << ", " << dist_pcl_tree[0] << "\n";
    }

    return 0;
}