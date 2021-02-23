/**
 * search=kdtree point=0.1,-0.1,0 k=5
 * search=kdtree point=0.1,-0.1,0 radius=0.5
 **/
#ifndef __KDTREE_ACTION_H__
#define __KDTREE_ACTION_H__

#include <pcl/kdtree/kdtree_flann.h>

class KdTreeAction : public SearchAction
{
    int K_;
    double radius_;
    double point_[3];

    template <typename PointT>
    int doitAsType(const PointCloud2 &cloud2_in, PointCloud2& cloud2_out);
public:
    KdTreeAction(std::vector<const char*>& actionStr);
    int doit(std::vector<PCW*>& in, std::vector<PCW*>& out);
    void dump() {
        pcl::console::print_highlight("Action: ");
        pcl::console::print_value("%s=%s\n", getClassName().c_str(), getName().c_str());
        pcl::console::print_info("Parameters: ");
        pcl::console::print_value("point=(%g,%g,%g), K=%d, radius=%g\n",
                point_[0], point_[1], point_[2], K_, radius_);
    }
};

KdTreeAction::KdTreeAction(std::vector<const char*>& actionStr)
:SearchAction("KdTree")
{
    point_[0] = point_[1] = point_[2] = 0.0f;
    radius_ = 0.0f;
    K_ = 0;

    for (size_t i=0; i<actionStr.size(); i++)
    {
        char key[64];
        char value[64];
        if (sscanf(actionStr[i], "%[^=]=%s", key, value) == 2) {
            if (!strcasecmp(key, "point")) {
                parse_arguments_3(value, point_[0], point_[1], point_[2]);
            } else if (!strcasecmp(key, "radius")) {
                radius_ = atof(value);
            } else if (!strcasecmp(key, "k")) {
                K_ = atoi(value);
            }
        }
    }
}

template <typename PointT>
int KdTreeAction::doitAsType(const PointCloud2 &cloud2_in, PointCloud2& cloud2_out)
{
    pcl::KdTreeFLANN<PointT> kdtree;

    typename pcl::PointCloud<PointT>::Ptr cloud_in (new pcl::PointCloud<PointT>);
    typename pcl::PointCloud<PointT>::Ptr cloud_out (new pcl::PointCloud<PointT>);
    pcl::fromPCLPointCloud2(cloud2_in, *cloud_in);
    kdtree.setInputCloud (cloud_in);

    PointT searchPoint;

    searchPoint.x = point_[0];
    searchPoint.y = point_[1];
    searchPoint.z = point_[2];

    std::vector<int> inliers;
    std::vector<float> distance;
    int neighbors = 0;

    if (K_ > 0) {
        // K nearest neighbor search
        inliers.reserve(K_);
        distance.reserve(K_);
        neighbors = kdtree.nearestKSearch (searchPoint, K_, inliers, distance);
    } else {
        // Neighbors within radius search
        neighbors = kdtree.radiusSearch (searchPoint, radius_, inliers, distance);
    }

    if (neighbors > 0) {
        pcl::copyPointCloud (*cloud_in, inliers, *cloud_out);
        pcl::toPCLPointCloud2(*cloud_out, cloud2_out);
    }

    return 0;
}

int KdTreeAction::doit(std::vector<PCW*>& in, std::vector<PCW*>& out)
{
    bool has_rgb, has_normals;
    parseCloudField(in[0]->cloud, has_rgb, has_normals);

    // 根据具体点云数据类型进行处理
    if (!has_rgb && !has_normals)
        doitAsType<pcl::PointXYZ> (*(in[0]->cloud), *(out[0]->cloud));
    else if (has_rgb && !has_normals)
        doitAsType<pcl::PointXYZRGB> (*(in[0]->cloud), *(out[0]->cloud));
    else  if (!has_rgb && has_normals)
        doitAsType<pcl::PointNormal> (*(in[0]->cloud), *(out[0]->cloud));
    else
        doitAsType<pcl::PointXYZRGBNormal> (*(in[0]->cloud), *(out[0]->cloud));

    // 输入点云已经没有用，可释放掉
    in[0]->clear();

    return 0;
}

#endif //__KDTREE_ACTION_H__