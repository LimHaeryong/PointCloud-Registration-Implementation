#include "Registration/icp.h"

#include <chrono>

ICP::ICP()
    : source_cloud_(new PointCloudT)
    , target_cloud_(new PointCloudT)
    , total_transform_(Eigen::Matrix4d::Identity())
    , kdtree_(new pcl::KdTreeFLANN<PointT>)
    , max_correspondence_distance_(100.0)
    , transformation_epsilon_(1e-6)
    , euclidean_fitness_epsilon_(1e-8)
    , max_iteration_(200)
{}

ICP::~ICP()
{
}

void ICP::setMaxCorrespondenceDistance(double max_correspondence_distance)
{
    max_correspondence_distance_ = max_correspondence_distance;
}

void ICP::setTransformationEpsilon(double transformation_epsilon)
{
    transformation_epsilon_ = transformation_epsilon;
}

void ICP::setEuclideanFitnessEpsilon(double euclidean_fitness_epsilon)
{
    euclidean_fitness_epsilon_ = euclidean_fitness_epsilon;
}

void ICP::setMaximumIterations(int max_iteration)
{
    max_iteration_ = max_iteration;
}

void ICP::setInputSource(const PointCloudPtr &cloud)
{
    source_cloud_ = cloud;
}
void ICP::setInputTarget(const PointCloudPtr &cloud)
{
    target_cloud_ = cloud;
}

Eigen::Matrix4d ICP::getFinalTransformation()
{
    return total_transform_;
}

void ICP::align(PointCloudT &output)
{
    total_transform_ = Eigen::Matrix4d::Identity();
    converged_ = false;
    std::size_t cloud_size = source_cloud_->size();
    pcl::transformPointCloud(*source_cloud_, output, total_transform_);
    kdtree_->setInputCloud(target_cloud_);

    std::vector<int> associations;
    std::vector<PointPair> point_pairs;

    int iteration(0);

    while (!converged_ && iteration++ < max_iteration_)
    {
        associate(output, associations);
        getPairPoints(output, associations, point_pairs);
        Eigen::Matrix4d transform = computeTransform(point_pairs);
        total_transform_ *= transform;
        pcl::transformPointCloud(*source_cloud_, output, total_transform_);
        if (transform.isApprox(Eigen::Matrix4d::Identity(), transformation_epsilon_))
        {
            converged_ = true;
        }
        if (computeEuclideanError(point_pairs, transform) < euclidean_fitness_epsilon_)
        {
            converged_ = true;
        }
    }
}

bool ICP::hasConverged()
{
    return converged_;
}

void ICP::associate(const PointCloudT &output, std::vector<int> &associations)
{
    size_t output_size = output.size();
    associations.clear();
    associations.resize(output_size, -1);

#pragma omp parallel for
    for (int i = 0; i < output_size; ++i)
    {
        std::vector<int> k_indices(1);
        std::vector<float> k_distances(1);
        if (kdtree_->nearestKSearch(output, i, 1, k_indices, k_distances) > 0 && k_distances[0] < max_correspondence_distance_)
        {
            associations[i] = k_indices[0];
        }
    }
}

void ICP::getPairPoints(const PointCloudT &output, const std::vector<int> &associations, std::vector<PointPair> &point_pairs)
{
    point_pairs.clear();
    point_pairs.reserve(associations.size());

    int i = 0;
    for (PointT point : output.points)
    {
        int j = associations[i];
        if (j >= 0)
        {
            PointT target_point = target_cloud_->points[j];
            point_pairs.emplace_back(point, target_point);
        }
        ++i;
    }
}

Eigen::Matrix4d ICP::computeTransform(const std::vector<PointPair> &point_pairs)
{
    std::size_t pair_size = point_pairs.size();

    Eigen::MatrixXd X(3, pair_size);
    Eigen::MatrixXd Y(3, pair_size);
    Eigen::MatrixXd P = Eigen::MatrixXd::Zero(3, 1);
    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(3, 1);

#pragma omp parallel for reduction(matrix_reduction : P, Q)
    for (int i = 0; i < pair_size; ++i)
    {
        P(0, 0) += point_pairs[i].p1.x;
        P(1, 0) += point_pairs[i].p1.y;
        P(2, 0) += point_pairs[i].p1.z;

        Q(0, 0) += point_pairs[i].p2.x;
        Q(1, 0) += point_pairs[i].p2.y;
        Q(2, 0) += point_pairs[i].p2.z;
    }

    P.block<3, 1>(0, 0) /= static_cast<double>(pair_size);
    Q.block<3, 1>(0, 0) /= static_cast<double>(pair_size);

#pragma omp parallel for
    for (std::size_t i = 0; i < pair_size; ++i)
    {
        X(0, i) = point_pairs[i].p1.x - P(0, 0);
        X(1, i) = point_pairs[i].p1.y - P(1, 0);
        X(2, i) = point_pairs[i].p1.z - P(2, 0);

        Y(0, i) = point_pairs[i].p2.x - Q(0, 0);
        Y(1, i) = point_pairs[i].p2.y - Q(1, 0);
        Y(2, i) = point_pairs[i].p2.z - Q(2, 0);
    }

    Eigen::Matrix3d S = X * Y.transpose();
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(S, Eigen::ComputeFullV | Eigen::ComputeFullU);
    Eigen::Matrix3d Sigma = Eigen::MatrixXd::Identity(3, 3);

    // fix determinent to 1
    Sigma(2, 2) = (svd.matrixV() * svd.matrixU().transpose()).determinant();

    Eigen::Matrix3d R = svd.matrixV() * Sigma * svd.matrixU().transpose();
    Eigen::Vector3d t = Q - R * P;

    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    transform.block<3, 3>(0, 0) = R;
    transform.block<3, 1>(0, 3) = t;

    return transform;
}

double ICP::computeEuclideanError(const std::vector<PointPair> &point_pairs, const Eigen::Matrix4d &transform)
{
    double error = 0.0;

#pragma omp parallel for reduction(+ : error)
    for (int i = 0; i < point_pairs.size(); ++i)
    {
        Eigen::Vector4d P = Eigen::Vector4d::Ones();
        Eigen::Vector4d Q = Eigen::Vector4d::Ones();

        PointPair point_pair = point_pairs[i];

        P(0, 0) = point_pair.p1.x;
        P(1, 0) = point_pair.p1.y;
        P(2, 0) = point_pair.p1.z;

        Q(0, 0) = point_pair.p2.x;
        Q(1, 0) = point_pair.p2.y;
        Q(2, 0) = point_pair.p2.z;

        P = transform * P;
        error += (P - Q).norm();
    }

    error /= static_cast<double>(point_pairs.size());

    return error;
}