#ifndef ICP_H_
#define ICP_H_

#include <iostream>
#include <omp.h>
#include <vector>

#include <Eigen/Core>
#include <Eigen/SVD>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>

#include "Registration/type.h"

#pragma omp declare reduction(matrix_reduction : Eigen::MatrixXd : omp_out += omp_in) initializer(omp_priv = Eigen::MatrixXd::Zero(omp_orig.rows(), omp_orig.cols()))

class ICP
{
public:
    using PointT = pcl::PointXYZ;
    using PointCloudT = pcl::PointCloud<PointT>;
    using PointCloudPtr = pcl::PointCloud<PointT>::Ptr;

    ICP();
    virtual ~ICP();

    void setMaxCorrespondenceDistance(double max_correspondence_distance);
    void setTransformationEpsilon(double transformation_epsilon);
    void setEuclideanFitnessEpsilon(double euclidean_fitness_epsilon);
    void setMaximumIterations(int max_iteration);

    void setInputSource(const PointCloudPtr& cloud);
    void setInputTarget(const PointCloudPtr& cloud);

    Eigen::Matrix4d getFinalTransformation();
    bool hasConverged();

    void align(PointCloudT& output);

protected:
    void associate(const PointCloudT& output, std::vector<int>& associations);
    void getPairPoints(const PointCloudT& output, const std::vector<int>& associations, std::vector<PointPair>& point_pairs);
    Eigen::Matrix4d computeTransform(const std::vector<PointPair>& point_pairs);
    double computeEuclideanError(const std::vector<PointPair>& point_pairs, const Eigen::Matrix4d& transform);

private:
    pcl::KdTreeFLANN<PointT>::Ptr kdtree_;
    PointCloudPtr source_cloud_;
    PointCloudPtr target_cloud_;

    Eigen::Matrix4d total_transform_;

    double max_correspondence_distance_;
    double euclidean_fitness_epsilon_;
    double transformation_epsilon_;
    int max_iteration_;
    bool converged_;

};


#endif // ICP_H_