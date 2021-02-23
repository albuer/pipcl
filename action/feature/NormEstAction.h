/**
 * feature=normal radius=0.03
 **/

#ifndef __NORM_EST_ACTION_H__
#define __NORM_EST_ACTION_H__

#include <pcl/features/normal_3d.h>

class NormEstAction : public FeatureAction
{
    double radius_;
    int K_;

    template <typename PointT>
    int doitAsType(const PointCloud2 &cloud2_in, PointCloud2& cloud2_out);
public:
    NormEstAction(std::vector<const char*>& vector);
    int doit(std::vector<PCW*>& in, std::vector<PCW*>& out);
    void dump() {
        pcl::console::print_highlight("Action: ");
        pcl::console::print_value("%s=%s\n", getClassName().c_str(), getName().c_str());
        pcl::console::print_info("Parameters: ");
        pcl::console::print_value("K=%d, radius=%g\n", K_, radius_);
    }
};

NormEstAction::NormEstAction(std::vector<const char*>& actionStr)
:FeatureAction("NormalEstimation")
{
    radius_ = 0.0f;
    K_ = 0;

    for (size_t i=0; i<actionStr.size(); i++) {
        char key[64];
        char value[64];
        if (sscanf(actionStr[i], "%[^=]=%s", key, value) == 2) {
            if (!strcasecmp(key, "radius")) {
                radius_ = atof(value);
            } else if (!strcasecmp(key, "K")) {
                K_ = atoi(value);
            }
        }
    }
}

template <typename PointT>
int NormEstAction::doitAsType(const PointCloud2 &cloud2_in, PointCloud2& cloud2_out)
{
    typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    fromPCLPointCloud2(cloud2_in, *cloud);

    // Create the normal estimation class, and pass the input dataset to it
    pcl::NormalEstimation<PointT, pcl::Normal> ne;
    ne.setInputCloud(cloud);

    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
    ne.setSearchMethod(tree);

    // Output datasets
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);

    if (K_ > 0) ne.setKSearch(K_);
    if (radius_ > 0.0f) ne.setRadiusSearch(radius_);

    // Compute the features
    ne.compute(*cloud_normals);

    toPCLPointCloud2(*cloud_normals, cloud2_out);

    return 0;
}

int NormEstAction::doit(std::vector<PCW*>& in, std::vector<PCW*>& out)
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

#endif //__NORM_EST_ACTION_H__