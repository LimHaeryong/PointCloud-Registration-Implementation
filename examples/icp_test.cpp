#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "Registration/icp.h"

using namespace std;

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

int main()
{
    cout << "icp_test started" << endl;

    // load pcd files
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    if(pcl::io::loadPCDFile<pcl::PointXYZ>("../resources/source.pcd", *source_cloud) == -1)
    {
        PCL_ERROR("Couldn't read pcd file \n");
        return -1;
    }

    if(pcl::io::loadPCDFile<pcl::PointXYZ>("../resources/target.pcd", *target_cloud) == -1)
    {
        PCL_ERROR("Couldn't read pcd file \n");
        return -1;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    ICP icp = ICP();
    icp.setInputSource(source_cloud);
    icp.setInputTarget(target_cloud);
    icp.align(*aligned_cloud);

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

    colorize(*source_cloud, *source_colored, {255, 0, 0});
    colorize(*aligned_cloud, *aligned_colored, {0, 255, 0});
    colorize(*target_cloud, *target_colored, {0, 0, 255});


    pcl::visualization::CloudViewer cloud_viewer("Cloud Viewer");
    cloud_viewer.showCloud(source_colored, "source");
    cloud_viewer.showCloud(aligned_colored, "align");
    cloud_viewer.showCloud(target_colored, "target");
    while(!cloud_viewer.wasStopped())
    {
        continue;
    }

    return 0;
}