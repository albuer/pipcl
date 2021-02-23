/**
 * PFH:  Point Feature Histograms
 *       feature=pfh n_k=1 k_radius=0.5
 * FPFH: Fast Point Feature Histograms
 *       feature=fpfh n_k=1 k_radius=0.5
 * VFH:  Viewpoint Feature Histogram
 *       feature=vfh n_k=1 k_radius=0.5
 */

#ifndef __FEATURE_ACTION_H__
#define __FEATURE_ACTION_H__

#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/pfh.h>
#include <pcl/features/vfh.h>

class FeatureAction : public Action
{
protected:
    double n_radius_, f_radius_;
    int n_K_, f_K_;

public:
    FeatureAction(std::string n):Action(n, Action::FEATURE) {}
    virtual int doit(std::vector<PCW*>& in, std::vector<PCW*>& out);
    static std::vector<Action*> parse(std::vector<const char*>& actionStr, int batch);
    static void showHelp(void);
    template <typename FeatureAlgorithm, typename PointIn, typename PointOut>
    void computeFeatureViaNormals(const PointCloud2 &input, PointCloud2 &output);
    void init(std::vector<const char*>& actionStr);
    void dump() {
        pcl::console::print_highlight("Action: ");
        pcl::console::print_value("%s=%s\n", getClassName().c_str(), getName().c_str());
        pcl::console::print_info("Parameters: ");
        pcl::console::print_value("normal K=%d, normal radius=%g, feature K=%d, feature radius=%g\n",
                n_K_, n_radius_, f_K_, f_radius_);
    }
};

class PfhAction : public FeatureAction
{
public:
    PfhAction(std::vector<const char*>& actionStr):FeatureAction("PFH") { init(actionStr); }
};

class FpfhAction : public FeatureAction
{
public:
    FpfhAction(std::vector<const char*>& actionStr):FeatureAction("FPFH") { init(actionStr); }
};

class VfhAction : public FeatureAction
{
public:
    VfhAction(std::vector<const char*>& actionStr):FeatureAction("VFH") { init(actionStr); }
};

#include "NormEstAction.h"

void FeatureAction::init(std::vector<const char*>& actionStr)
{
    n_K_ = f_K_ = 0;
    n_radius_ = f_radius_ = 0.0f;

    for (size_t i=0; i<actionStr.size(); i++) {
        char key[64];
        char value[64];
        if (sscanf(actionStr[i], "%[^=]=%s", key, value) == 2) {
            if (!strcasecmp(key, "n_radius")) {
                n_radius_ = atof(value);
            } else if (!strcasecmp(key, "f_radius")) {
                f_radius_ = atof(value);
            } else if (!strcasecmp(key, "n_K")) {
                n_K_ = atoi(value);
            } else if (!strcasecmp(key, "f_K")) {
                f_K_ = atoi(value);
            }
        }
    }
}

template <typename FeatureAlgorithm, typename PointIn, typename PointOut>
void FeatureAction::computeFeatureViaNormals(const PointCloud2 &input, PointCloud2 &output)
{
    typename pcl::PointCloud<PointIn>::Ptr xyz(new pcl::PointCloud<PointIn>);
    fromPCLPointCloud2(input, *xyz);

    pcl::NormalEstimation<PointIn, pcl::Normal> ne;
    ne.setInputCloud(xyz);
    ne.setSearchMethod(typename pcl::search::KdTree<PointIn>::Ptr(new pcl::search::KdTree<PointIn>));
    if (n_K_>0) ne.setKSearch(n_K_);
    if (n_radius_>0.0) ne.setRadiusSearch(n_radius_);

    typename pcl::PointCloud<pcl::Normal>::Ptr normals = typename pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>);
    ne.compute(*normals);

    FeatureAlgorithm feature_est;
    feature_est.setInputCloud(xyz);
    feature_est.setInputNormals(normals);

    feature_est.setSearchMethod(typename pcl::search::KdTree<PointIn>::Ptr(new pcl::search::KdTree<PointIn>));

    pcl::PointCloud<PointOut> output_features;

    if (f_K_>0) feature_est.setKSearch(f_K_);
    if (f_radius_>0.0) feature_est.setRadiusSearch(f_radius_);

    feature_est.compute(output_features);

    // Convert data back
    toPCLPointCloud2(output_features, output);
}

int FeatureAction::doit(std::vector<PCW*>& in, std::vector<PCW*>& out)
{
    bool has_rgb, has_normals;
    parseCloudField(in[0]->cloud, has_rgb, has_normals);

    if (!strncasecmp(getName().c_str(), "PFH", 3)) {
        // 根据具体点云数据类型进行处理
        if (!has_rgb)
            computeFeatureViaNormals<pcl::PFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::PFHSignature125>, pcl::PointXYZ, pcl::PFHSignature125>(*(in[0]->cloud), *(out[0]->cloud));
        else
            computeFeatureViaNormals<pcl::PFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::PFHSignature125>, pcl::PointXYZRGB, pcl::PFHSignature125>(*(in[0]->cloud), *(out[0]->cloud));
    } else if (!strncasecmp(getName().c_str(), "FPFH", 4)) {
        // 根据具体点云数据类型进行处理
        if (!has_rgb)
            computeFeatureViaNormals<pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33>, pcl::PointXYZ, pcl::FPFHSignature33>(*(in[0]->cloud), *(out[0]->cloud));
        else
            computeFeatureViaNormals<pcl::FPFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33>, pcl::PointXYZRGB, pcl::FPFHSignature33>(*(in[0]->cloud), *(out[0]->cloud));
    } else if (!strncasecmp(getName().c_str(), "VFH", 3)) {
        // 根据具体点云数据类型进行处理
        if (!has_rgb)
            computeFeatureViaNormals<pcl::VFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308>, pcl::PointXYZ, pcl::VFHSignature308>(*(in[0]->cloud), *(out[0]->cloud));
        else
            computeFeatureViaNormals<pcl::VFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::VFHSignature308>, pcl::PointXYZRGB, pcl::VFHSignature308>(*(in[0]->cloud), *(out[0]->cloud));
    }

    // 输入点云已经没有用，可释放掉
    in[0]->clear();

    return 0;
}

std::vector<Action*> FeatureAction::parse(std::vector<const char*>& actionStr, int batch) {
    std::vector<Action*> actionList;
    const char* feature = actionStr.front()+strlen("feature=");

    // todo: 判断filter类型
    FeatureAction* action;
    for (int i=0; i<batch; i++) {
        if (!strcasecmp(feature, "normal")) {
            action = new NormEstAction(actionStr);
        } else if (!strcasecmp(feature, "PFH")) {
            action = new PfhAction(actionStr);
        } else if (!strcasecmp(feature, "FPFH")) {
            action = new FpfhAction(actionStr);
        } else if (!strcasecmp(feature, "VFH")) {
            action = new VfhAction(actionStr);
        } else {
            action = NULL;
        }

        if (action != NULL)
            actionList.push_back(action);
    }

    return actionList;
}

void FeatureAction::showHelp(void)
{
    printf("\n");
    printf("feature=<ACTION> [ARGS]\n");
    printf("\n");
    printf("All feature actions are:\n");
    printf("\n");
    printf("  Normal             Normal Estimation\n");
    printf("    radius=0.0         the sphere radius used as the maximum distance to consider a point a neighbor\n");
    printf("    K=0                the number of k-nearest neighbors\n");
    printf("\n");
    printf("  PFH                Point Feature Histogram\n");
    printf("  FPFH               Fast Point Feature Histogram\n");
    printf("  VFH                Viewpoint Feature Histogram\n");
    printf("    n_K=0              normal estimation k-nearest\n");
    printf("    n_radius=0.0       normal estimation radius\n");
    printf("    f_K=0              feature estimation k-nearest\n");
    printf("    f_radius=0.0       feature estimation radius\n");
    printf("\n");
}

#endif //__FEATURE_ACTION_H__