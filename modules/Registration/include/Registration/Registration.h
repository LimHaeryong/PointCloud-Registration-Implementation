#ifndef REGISTRATION_H_
#define REGISTRATION_H_

#include <iostream>
#include <omp.h>
#include <vector>

#include <Eigen/Core>
#include <Eigen/SVD>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>

#pragma omp declare reduction(matrix_reduction : Eigen::MatrixXd : omp_out += omp_in) initializer(omp_priv = Eigen::MatrixXd::Zero(omp_orig.rows(), omp_orig.cols()))

class Registration
{
public:
    using PointT = pcl::PointXYZ;
    using PointCloudT = pcl::PointCloud<PointT>;
    using PointCloudPtr = pcl::PointCloud<PointT>::Ptr;

    Registration();
    virtual ~Registration();

    void setInputSource(const PointCloudPtr& cloud);
    void setInputTarget(const PointCloudPtr& cloud);

    Eigen::Matrix4d getFinalTransformation();
    bool hasConverged();

    virtual void align(PointCloudT& output){}

protected:
    PointCloudPtr source_cloud_;
    PointCloudPtr target_cloud_;

    Eigen::Matrix4d total_transform_;
    bool converged_;
    
private:

};

#endif // REGISTRATION_H_