#include "Registration/Registration.h"

Registration::Registration()
    : source_cloud_ (new PointCloudT)
    , target_cloud_ (new PointCloudT)
    , total_transform_ (Eigen::Matrix4d::Identity())
{
}

Registration::~Registration()
{
}

void Registration::setInputSource(const PointCloudPtr& cloud)
{
    source_cloud_ = cloud;
}
void Registration::setInputTarget(const PointCloudPtr& cloud)
{
    target_cloud_ = cloud;
}

Eigen::Matrix4d Registration::getFinalTransformation()
{
    return total_transform_;
}

bool Registration::hasConverged()
{
    return converged_;
}