#ifndef ICP_H_
#define ICP_H_

#include "Registration/type.h"
#include "Registration/registration.h"

#pragma omp declare reduction(matrix_reduction : Eigen::MatrixXd : omp_out += omp_in) initializer(omp_priv = Eigen::MatrixXd::Zero(omp_orig.rows(), omp_orig.cols()))

template <typename PointT>
class ICP : public Registration<PointT>
{
public:
    using PointCloudT = typename pcl::PointCloud<PointT>;
    using PointCloudPtr = typename pcl::PointCloud<PointT>::Ptr;

    struct PointPair
    {
        PointT p1;
        PointT p2;

    PointPair(const PointT& p1, const PointT& p2)
        : p1(p1), p2(p2)
        {}
    };

    ICP()
        : max_correspondence_distance_(100.0)
        , transformation_epsilon_(1e-6)
        , euclidean_fitness_epsilon_(1e-8)
    {}

    void setMaxCorrespondenceDistance(double max_correspondence_distance)
    {
        max_correspondence_distance_ = max_correspondence_distance;
    }
    void setTransformationEpsilon(double transformation_epsilon)
    {
        transformation_epsilon_ = transformation_epsilon;
    }
    void setEuclideanFitnessEpsilon(double euclidean_fitness_epsilon)
    {
        euclidean_fitness_epsilon_ = euclidean_fitness_epsilon;
    }

    void align(PointCloudT& output) override;

private:
    void associate(const PointCloudT& output, std::vector<int>& associations);
    void getPairPoints(const PointCloudT& output, const std::vector<int>& associations, std::vector<PointPair>& point_pairs);
    Eigen::Matrix4d computeTransform(const std::vector<PointPair>& point_pairs);
    double computeEuclideanError(const std::vector<PointPair>& point_pairs, const Eigen::Matrix4d& transform);

    double max_correspondence_distance_;
    double euclidean_fitness_epsilon_;
    double transformation_epsilon_;
};


#endif // ICP_H_