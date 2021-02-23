/**
 * StatisticalOutlierRemoval (SOR) filter
 * filter=SOR meank=50 stdmul=1.0
 **/

#ifndef __SOR_ACTION_H__
#define __SOR_ACTION_H__

#include <pcl/filters/statistical_outlier_removal.h>

class SorAction : public FilterAction
{
    int mean_k_;
    double std_mul_;

    template <typename PointT>
    int doitAsType(const PointCloud2 &cloud2, PointCloud2& inlier, PointCloud2& outlier);
public:
    SorAction(std::vector<const char*>& vector);
    int doit(std::vector<PCW*>& in, std::vector<PCW*>& out);
    void dump() {
        pcl::console::print_highlight("Action: ");
        pcl::console::print_value("%s=%s\n", getClassName().c_str(), getName().c_str());
        pcl::console::print_info("Parameters: ");
        pcl::console::print_value("MeanK=%d, StddevMul=%g\n", mean_k_, std_mul_);
    }
};

SorAction::SorAction(std::vector<const char*>& actionStr)
:FilterAction("SOR")
{
    mean_k_ = 2;
    std_mul_ = 0.0f;

    for (size_t i=0; i<actionStr.size(); i++) {
        char key[64];
        char value[64];
        if (sscanf(actionStr[i], "%[^=]=%s", key, value) == 2) {
            if (!strcasecmp(key, "meank")) {
                mean_k_ = atoi(value);
            } else if (!strcasecmp(key, "stdmul")) {
                std_mul_ = atof(value);
            }
        }
    }
}

template <typename PointT>
int SorAction::doitAsType(const PointCloud2 &cloud2, PointCloud2& inlier, PointCloud2& outlier)
{
    typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT> cloud_inlier, cloud_outlier;
    fromPCLPointCloud2(cloud2, *cloud);

    pcl::StatisticalOutlierRemoval<PointT> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(mean_k_);
    sor.setStddevMulThresh(std_mul_);

    sor.setNegative(false);
    sor.filter(cloud_inlier);
    toPCLPointCloud2(cloud_inlier, inlier);

    sor.setNegative(true);
    sor.filter(cloud_outlier);
    toPCLPointCloud2(cloud_outlier, outlier);

    return 0;
}

int SorAction::doit(std::vector<PCW*>& in, std::vector<PCW*>& out)
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

#endif //__SOR_ACTION_H__