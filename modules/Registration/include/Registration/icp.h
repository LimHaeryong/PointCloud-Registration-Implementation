#ifndef ICP_H_
#define ICP_H_

#include "Registration/type.h"
#include "Registration/Registration.h"

class ICP : public Registration
{
public:
    ICP();
    virtual ~ICP();

    void setMaxCorrespondenceDistance(double max_correspondence_distance);
    void setTransformationEpsilon(double transformation_epsilon);
    void setEuclideanFitnessEpsilon(double euclidean_fitness_epsilon);
    void setMaximumIterations(int max_iteration);
    void align(PointCloudT& output);

private:
    pcl::KdTreeFLANN<PointT>::Ptr kdtree_;
    double max_correspondence_distance_;
    double euclidean_fitness_epsilon_;
    double transformation_epsilon_;
    int max_iteration_;

    void associate(const PointCloudT& output, std::vector<int>& associations);
    void getPairPoints(const PointCloudT& output, const std::vector<int>& associations, std::vector<PointPair>& point_pairs);
    Eigen::Matrix4d computeTransform(const std::vector<PointPair>& point_pairs);
    double computeEuclideanError(const std::vector<PointPair>& point_pairs, const Eigen::Matrix4d& transform);
};


#endif // ICP_H_