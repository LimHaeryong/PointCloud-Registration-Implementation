#ifndef TYPE_H_
#define TYPE_H_

#include <vector>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>


struct PointPair
{
    pcl::PointXYZ p1;
    pcl::PointXYZ p2;

    PointPair(const pcl::PointXYZ& p1, const pcl::PointXYZ& p2)
    : p1(p1)
    , p2(p2)
    {}
};

#endif // TYPE_H_