#ifndef _REGISTRATION_REGISTRATION_H_
#define _REGISTRATION_REGISTRATION_H_

#include <iostream>
#include <omp.h>
#include <vector>

#include <Eigen/Core>
#include <Eigen/SVD>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>

template <typename PointT> 
class Registration
{
public:
    using PointCloudT = typename pcl::PointCloud<PointT>;
    using PointCloudPtr = typename pcl::PointCloud<PointT>::Ptr;

    Registration()
        : source_cloud_(new PointCloudT)
        , target_cloud_(new PointCloudT)
        , total_transform_(Eigen::Matrix4d::Identity())
        , kdtree_(new pcl::KdTreeFLANN<PointT>)
        , max_iteration_(200)
    {}

    void setMaximumIterations(int max_iteration)
    {
        max_iteration_ = max_iteration;
    }

    void setInputSource(const PointCloudPtr& cloud)
    {
        source_cloud_ = cloud;
    }

    void setInputTarget(const PointCloudPtr& cloud)
    {
        target_cloud_ = cloud;
    }

    Eigen::Matrix4d getFinalTransformation() const
    {
        return total_transform_;
    }

    bool hasConverged() const { return converged_; }

    virtual void align(PointCloudT& output) = 0;

protected:
    PointCloudPtr source_cloud_;
    PointCloudPtr target_cloud_;

    Eigen::Matrix4d total_transform_;
    typename pcl::KdTreeFLANN<PointT>::Ptr kdtree_;
    
    int max_iteration_;
    bool converged_;

};

#endif // _REGISTRATION_REGISTRATION_H_