/**
 * RadiusOutlier Removal (ROR) filter
 * filter=ROR radius=0.8 minpts=2
 **/

#ifndef __ROR_ACTION_H__
#define __ROR_ACTION_H__

#include <pcl/filters/radius_outlier_removal.h>

class RorAction : public FilterAction
{
    int min_pts_;
    double radius_;

    template <typename PointT>
    int doitAsType(const PointCloud2 &cloud2, PointCloud2& inlier, PointCloud2& outlier);
public:
    RorAction(std::vector<const char*>& vector);
    int doit(std::vector<PCW*>& in, std::vector<PCW*>& out);
    void dump() {
        pcl::console::print_highlight("Action: ");
        pcl::console::print_value("%s=%s\n", getClassName().c_str(), getName().c_str());
        pcl::console::print_info("Parameters: ");
        pcl::console::print_value("radius=%g, min points=%d\n", radius_, min_pts_);
    }
};

RorAction::RorAction(std::vector<const char*>& actionStr)
:FilterAction("SOR")
{
    min_pts_ = 2;
    radius_ = 0.8f;

    for (size_t i=0; i<actionStr.size(); i++) {
        char key[64];
        char value[64];
        if (sscanf(actionStr[i], "%[^=]=%s", key, value) == 2) {
            if (!strcasecmp(key, "minpts")) {
                min_pts_ = atoi(value);
            } else if (!strcasecmp(key, "radius")) {
                radius_ = atof(value);
            }
        }
    }
}

template <typename PointT>
int RorAction::doitAsType(const PointCloud2 &cloud2, PointCloud2& inlier, PointCloud2& outlier)
{
    typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT> cloud_inlier, cloud_outlier;
    fromPCLPointCloud2(cloud2, *cloud);

    pcl::RadiusOutlierRemoval<PointT> ror;
    ror.setInputCloud(cloud);
    ror.setRadiusSearch(radius_);
    ror.setMinNeighborsInRadius(min_pts_);
    ror.setKeepOrganized(true);

    ror.setNegative(false);
    ror.filter(cloud_inlier);
    toPCLPointCloud2(cloud_inlier, inlier);

    ror.setNegative(true);
    ror.filter(cloud_outlier);
    toPCLPointCloud2(cloud_outlier, outlier);

    return 0;
}

int RorAction::doit(std::vector<PCW*>& in, std::vector<PCW*>& out)
{
    PointCloud2 inlier, outlier;

    bool has_rgb, has_normals;
    parseCloudField(in[0]->cloud, has_rgb, has_normals);

    // 根据具体点云数据类型进行处理
    if (!has_rgb && !has_normals)
        doitAsType<pcl::PointXYZ> (*(in[0]->cloud), inlier, outlier);
    else if (has_rgb && !has_normals)
        doitAsType<pcl::PointXYZRGB> (*(in[0]->cloud), inlier, outlier);
    else  if (!has_rgb && has_normals)
        doitAsType<pcl::PointNormal> (*(in[0]->cloud), inlier, outlier);
    else
        doitAsType<pcl::PointXYZRGBNormal> (*(in[0]->cloud), inlier, outlier);

    *(out[0]->cloud) = inlier;

    if (out.size() >= 2)
        *(out[1]->cloud) = outlier;

    // 输入点云已经没有用，可释放掉
    in[0]->clear();

    return 0;
}

#endif //__ROR_ACTION_H__