#ifndef _REGISTRATION_KDTREE_H_
#define _REGISTRATION_KDTREE_H_

#include <algorithm>
#include <cmath>
#include <memory>
#include <numeric>
#include <iostream>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

//template <typename PointT>
class KDTree
{
public:
    using PointT = typename pcl::PointXYZ;
    using PointCloudT = typename pcl::PointCloud<PointT>;
    using PointCloudPtr = typename pcl::PointCloud<PointT>::Ptr;

    KDTree()
        : cloud_(pcl::make_shared<PointCloudT>())
    {}

    enum class Dim
    {
        x = 0,
        y = 1,
        z = 2
    };

    struct Bound
    {
        float x_min = -std::numeric_limits<float>::infinity();
        float x_max = std::numeric_limits<float>::infinity();
        float y_min = -std::numeric_limits<float>::infinity();
        float y_max = std::numeric_limits<float>::infinity();
        float z_min = -std::numeric_limits<float>::infinity();
        float z_max = std::numeric_limits<float>::infinity();

        void setMin(Dim dim, float val)
        {
            switch(dim)
            {
            case Dim::x :
                x_min = val;
                break;
            case Dim::y :
                y_min = val;
                break;
            case Dim::z :
                z_min = val;
                break;
            }
        }

        void setMax(Dim dim, float val)
        {
            switch(dim)
            {
            case Dim::x :
                x_max = val;
                break;
            case Dim::y :
                y_max = val;
                break;
            case Dim::z :
                z_max = val;
                break;
            }
        }

        float getMin(Dim dim) const
        {
            float val;
            switch(dim)
            {
            case Dim::x :
                val = x_min;
                break;
            case Dim::y :
                val = y_min;
                break;
            case Dim::z :
                val = z_min;
                break;
            }
            return val;
        }

        float getMax(Dim dim) const
        {
            float val;
            switch(dim)
            {
            case Dim::x :
                val = x_max;
                break;
            case Dim::y :
                val = y_max;
                break;
            case Dim::z :
                val = z_max;
                break;
            }
            return val;
        }

        void printBound() const
        {
            std::cout << "Bound : " << x_min << " " << x_max << ", " << y_min << " " << y_max << ", " << z_min << " " << z_max << '\n';
        }
    };

    struct Node
    {
        int index = -1;
        std::shared_ptr<Node> left = nullptr;
        std::shared_ptr<Node> right = nullptr;

        Node(int index) : index(index) {}
    };
    using NodePtr = typename std::shared_ptr<Node>;

    void setInputCloud(PointCloudPtr cloud)
    { 
        cloud_ = cloud; 
        next_idx_ = cloud_->size();

        for(int i = 0; i < next_idx_; ++i)
        {
            root_ = insert(root_, i, Dim::x);
        }
    }

    void addPoint(const PointT& point)
    {
        cloud_->push_back(point);
        root_ = insert(root_, next_idx_++, Dim::x);
    }

    void printTree(std::shared_ptr<Node> node, int level = 0) const
    {
        if(level == 0)
        {
            std::cout << "----print tree----\n";
        }

        for(int i = 0; i < level; ++i)
        {
            std::cout << " ";
        }

        if(node != nullptr)
        {
            PointT& p = cloud_->points[node->index];
            std::cout << std::fixed << std::setprecision(2) << p.x << "," << p.y <<"," << p.z << '\n';

            printTree(node->left, level + 1);
            printTree(node->right, level + 1);
        }
        else
        {
            std::cout << "nullptr" << '\n';
        }
    }

    void nearestSearch(const PointT& target, float& dist_sq, int& idx)
    {
        idx = -1;
        dist_sq = std::numeric_limits<float>::infinity();
        nearestSearch(root_, target, Dim::x, Bound{}, dist_sq, idx);
    }

private:
    int next_idx_ = 0;
    PointCloudPtr cloud_;
    std::shared_ptr<Node> root_ = nullptr;

    NodePtr insert(NodePtr node, int val_idx, Dim dim)
    {
        if(node == nullptr)
        {
            return std::make_shared<Node>(val_idx);
        }
        
        PointT& p = cloud_->points[val_idx];
        PointT& p_node = cloud_->points[node->index];
        float val, val_node;
        switch(dim)
        {
        case Dim::x :
            val = p.x;
            val_node = p_node.x;
            break;
        case Dim::y :
            val = p.y;
            val_node = p_node.y;
            break;
        case Dim::z :
            val = p.z;
            val_node = p_node.z;
            break;
        }

        Dim next_dim = static_cast<Dim>((static_cast<int>(dim) + 1) % 3);
        if(val < val_node)
        {
            node->left = insert(node->left, val_idx, next_dim);
        }
        else if(val >= val_node)
        {
            node->right = insert(node->right, val_idx, next_dim);
        }

        return node;
    }

    void nearestSearch(NodePtr node, const PointT& p, Dim dim, Bound bound, float& best_dist, int& best_idx)
    {
        if(node == nullptr || distance(p, bound) > best_dist)
            return;

        const PointT& p_node = cloud_->points[node->index];
        float dist = distance(p, p_node);
        if(dist < best_dist)
        {
            best_dist = dist;
            best_idx = node->index;
        }

        float val, val_node, tmp;
        switch(dim)
        {
        case Dim::x :
            val = p.x;
            val_node = p_node.x;
            break;
        case Dim::y :
            val = p.y;
            val_node = p_node.y;
            break;
        case Dim::z :
            val = p.z;
            val_node = p_node.z;
            break;
        }

        Dim next_dim = static_cast<Dim>((static_cast<int>(dim) + 1) % 3);
        if(val < val_node)
        {
            tmp = bound.getMax(dim);
            bound.setMax(dim, val_node);
            nearestSearch(node->left, p, next_dim, bound, best_dist, best_idx);
            bound.setMax(dim, tmp);

            tmp = bound.getMin(dim);
            bound.setMin(dim, val_node);
            nearestSearch(node->right, p, next_dim, bound, best_dist, best_idx);
            bound.setMin(dim, tmp);
        }
        else if(val >= val_node)
        {
            tmp = bound.getMin(dim);
            bound.setMin(dim, val_node);
            nearestSearch(node->right, p, next_dim, bound, best_dist, best_idx);
            bound.setMin(dim, tmp);
            
            tmp = bound.getMax(dim);
            bound.setMax(dim, val_node);
            nearestSearch(node->left, p, next_dim, bound, best_dist, best_idx);
            bound.setMax(dim, tmp);
        }

    }

    float distance(const PointT& a, const PointT& b)
    {
        return std::pow(a.x - b.x, 2.0f) + std::pow(a.y - b.y, 2.0f) + std::pow(a.z - b.z, 2.0f);
    }

    float distance(const PointT& p, const Bound& bound)
    {
        float distance = 0.0f;

        if(p.x < bound.x_min) distance += std::pow(p.x - bound.x_min, 2.0f);
        else if(p.x > bound.x_max) distance += std::pow(p.x - bound.x_max, 2.0f);
        
        if(p.y < bound.y_min) distance += std::pow(p.y - bound.y_min, 2.0f);
        else if(p.y > bound.y_max) distance += std::pow(p.x - bound.y_max, 2.0f);
        
        if(p.z < bound.z_min) distance += std::pow(p.z - bound.z_min, 2.0f);
        else if(p.z > bound.x_max) distance += std::pow(p.z - bound.z_max, 2.0f);

        return distance;
    }
};

#endif // _REGISTRATION_KDTREE_H_