/**
 * segmentation=SAC model=plane threshold=0.01 iterations=1000
 **/

#ifndef __SAC_ACTION_H__
#define __SAC_ACTION_H__

#include <map>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

class SacAction : public SegmentAction
{
    int model_;
    int method_;
    int iterations_;
    double threshold_;

    std::map<int, std::string> sacmodel_map_;
    int getModel(const char* str);

    std::map<int, std::string> sacmethod_map_;
    int getMethod(const char* str);

    template <typename PointT>
    int doitAsType(const PointCloud2 &cloud2, PointCloud2& inlier, PointCloud2& outlier);
public:
    SacAction(std::vector<const char*>& vector);
    int doit(std::vector<PCW*>& in, std::vector<PCW*>& out);
    void dump() {
        pcl::console::print_highlight("Action: ");
        pcl::console::print_value("%s=%s\n", getClassName().c_str(), getName().c_str());
        pcl::console::print_info("Parameters: ");
        pcl::console::print_value("model=%s, method=%s, iterations=%d, threshold=%g\n",
                sacmodel_map_[model_].c_str(),
                sacmethod_map_[method_].c_str(),
                iterations_, threshold_);
    }
};

int SacAction::getModel(const char* str)
{
    int model = pcl::SACMODEL_PLANE;

    for(std::map<int, std::string>::iterator it = sacmodel_map_.begin(); it != sacmodel_map_.end(); it++)
    {
        if (!strcasecmp(str, it->second.c_str())) {
            model = it->first;
        }
    }

    return model;
}

int SacAction::getMethod(const char* str)
{
    int method = pcl::SAC_RANSAC;

    for(std::map<int, std::string>::iterator it = sacmethod_map_.begin(); it != sacmethod_map_.end(); it++)
    {
        if (!strcasecmp(str, it->second.c_str())) {
            method = it->first;
        }
    }

    return method;
}

SacAction::SacAction(std::vector<const char*>& actionStr)
:SegmentAction("SAC")
{
    // init Sac Model Map
    sacmodel_map_.insert(std::pair<int, std::string>(pcl::SACMODEL_PLANE, "PLANE"));
    sacmodel_map_.insert(std::pair<int, std::string>(pcl::SACMODEL_LINE, "LINE"));
    sacmodel_map_.insert(std::pair<int, std::string>(pcl::SACMODEL_CIRCLE2D, "CIRCLE2D"));
    sacmodel_map_.insert(std::pair<int, std::string>(pcl::SACMODEL_CIRCLE3D, "CIRCLE3D"));
    sacmodel_map_.insert(std::pair<int, std::string>(pcl::SACMODEL_SPHERE, "SPHERE"));
    sacmodel_map_.insert(std::pair<int, std::string>(pcl::SACMODEL_CYLINDER, "CYLINDER"));
    sacmodel_map_.insert(std::pair<int, std::string>(pcl::SACMODEL_CONE, "CONE"));
    sacmodel_map_.insert(std::pair<int, std::string>(pcl::SACMODEL_TORUS, "TORUS"));

    // init Sac Method Map
    sacmethod_map_.insert(std::pair<int, std::string>(pcl::SAC_RANSAC, "RANSAC"));
    sacmethod_map_.insert(std::pair<int, std::string>(pcl::SAC_LMEDS, "LMEDS"));
    sacmethod_map_.insert(std::pair<int, std::string>(pcl::SAC_MSAC, "MSAC"));
    sacmethod_map_.insert(std::pair<int, std::string>(pcl::SAC_RRANSAC, "RRANSAC"));
    sacmethod_map_.insert(std::pair<int, std::string>(pcl::SAC_RMSAC, "RMSAC"));
    sacmethod_map_.insert(std::pair<int, std::string>(pcl::SAC_MLESAC, "MLESAC"));
    sacmethod_map_.insert(std::pair<int, std::string>(pcl::SAC_PROSAC, "PROSAC"));

    model_ = pcl::SACMODEL_PLANE;
    method_ = pcl::SAC_RANSAC;
    threshold_ = 0.1f;
    iterations_ = 50;

    for (size_t i=0; i<actionStr.size(); i++) {
        char key[64];
        char value[64];
        if (sscanf(actionStr[i], "%[^=]=%s", key, value) == 2) {
            if (!strcasecmp(key, "model")) {
                model_ = getModel(value);
            } else if (!strcasecmp(key, "method")) {
                method_ = getMethod(value);
            } else if (!strcasecmp(key, "iterations")) {
                iterations_ = atoi(value);
            } else if (!strcasecmp(key, "threshold")) {
                threshold_ = atof(value);
            }
        }
    }
}

template <typename PointT>
int SacAction::doitAsType(const PointCloud2 &cloud2, PointCloud2& inlier, PointCloud2& outlier)
{
    typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT> cloud_inlier, cloud_outlier;
    fromPCLPointCloud2(cloud2, *cloud);

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(model_);
    seg.setMethodType(method_);
    seg.setDistanceThreshold(threshold_);
    seg.setMaxIterations(iterations_);

    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);

    extract.setNegative (false);
    extract.filter (cloud_inlier);
    toPCLPointCloud2(cloud_inlier, inlier);

    extract.setNegative (true);
    extract.filter (cloud_outlier);
    toPCLPointCloud2(cloud_outlier, outlier);

    return 0;
}

int SacAction::doit(std::vector<PCW*>& in, std::vector<PCW*>& out)
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

#endif //__SAC_ACTION_H__