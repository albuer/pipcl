/**
 * registration=ICP NonLinear=1 iterations=50 dist=0.05
 **/
#ifndef __ICP_ACTION_H__
#define __ICP_ACTION_H__

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/incremental_registration.h>

class IcpAction : public RegistrationAction
{
    bool nonLinear_;
    int iterations_;
    double dist_;
    double rans_;
    double epsilon_;

    template <typename PointT>
    int doitAsType(std::vector<PCW*>& in, PointCloud2& out);
public:
    IcpAction(std::vector<const char*>& vector);
    int doit(std::vector<PCW*>& in, std::vector<PCW*>& out);
    void dump() {
        pcl::console::print_highlight("Action: ");
        pcl::console::print_value("%s=%s\n", getClassName().c_str(), getName().c_str());
        pcl::console::print_info("Parameters: ");
        pcl::console::print_value("iterations=%d, distance=%g, RANS=%g, epsilon=%g, nonLinear=%d\n",
                iterations_, dist_, rans_, epsilon_, nonLinear_);
    }
};

IcpAction::IcpAction(std::vector<const char*>& actionStr)
:RegistrationAction("ICP")
{
    nonLinear_ = false;
    iterations_ = 50;
    dist_ = 0.05f;
    rans_ = 0.05f;
    epsilon_ = 0.000001f;

    for (size_t i=0; i<actionStr.size(); i++) {
        char key[64];
        char value[64];
        if (sscanf(actionStr[i], "%[^=]=%s", key, value) == 2) {
            if (!strcasecmp(key, "iterations")) {
                iterations_ = atoi(value);
            } else if (!strcasecmp(key, "dist")) {
                dist_ = atof(value);
            } else if (!strcasecmp(key, "rans")) {
                rans_ = atof(value);
            } else if (!strcasecmp(key, "epsilon")) {
                epsilon_ = atof(value);
            } else if (!strcasecmp(key, "NonLinear")) {
                nonLinear_ = !!atoi(value);
            }
        }
    }
}

template <typename PointT>
int IcpAction::doitAsType(std::vector<PCW*>& in, PointCloud2& out)
{
    typename pcl::IterativeClosestPoint<PointT, PointT>::Ptr icp;
    if (nonLinear_)
    {
        icp.reset(new pcl::IterativeClosestPointNonLinear<PointT, PointT>());
    }
    else
    {
        icp.reset(new pcl::IterativeClosestPoint<PointT, PointT>());
    }
    icp->setMaximumIterations (iterations_);
    icp->setMaxCorrespondenceDistance (dist_);
    icp->setRANSACOutlierRejectionThreshold (rans_);
    icp->setTransformationEpsilon (epsilon_);

    pcl::registration::IncrementalRegistration<PointT> iicp;
    iicp.setRegistration (icp);

    typename pcl::PointCloud<PointT>::Ptr cloud_out(new pcl::PointCloud<PointT>);
    for (size_t i = 0; i < in.size(); i++)
    {
        typename pcl::PointCloud<PointT>::Ptr data(new pcl::PointCloud<PointT>);
        fromPCLPointCloud2(*(in[i]->cloud), *data);
        if (!iicp.registerCloud(data))
        {
            pcl::console::print_warn("Registration failed. Resetting transform\n");
            iicp.reset();
            iicp.registerCloud(data);
        };

        typename pcl::PointCloud<PointT>::Ptr tmp(new pcl::PointCloud<PointT>);
        pcl::transformPointCloud(*data, *tmp, iicp.getAbsoluteTransform());
        std::cout << "Transform:\n" << iicp.getAbsoluteTransform() << std::endl;

        *cloud_out += *tmp;
    }

    toPCLPointCloud2(*cloud_out, out);

    return 0;
}

int IcpAction::doit(std::vector<PCW*>& in, std::vector<PCW*>& out)
{
    PointCloud2 inlier, outlier;

    bool has_rgb, has_normals;
    parseCloudField(in[0]->cloud, has_rgb, has_normals);

    // 根据具体点云数据类型进行处理
    if (!has_rgb && !has_normals)
        doitAsType<pcl::PointXYZ> (in, *(out[0]->cloud));
    else if (has_rgb && !has_normals)
        doitAsType<pcl::PointXYZRGB> (in, *(out[0]->cloud));
    else  if (!has_rgb && has_normals)
        doitAsType<pcl::PointNormal> (in, *(out[0]->cloud));
    else
        doitAsType<pcl::PointXYZRGBNormal> (in, *(out[0]->cloud));

    // 输入点云已经没有用，可释放掉
    for (size_t i = 0; i < in.size(); i++)
    {
        in[i]->clear();
    }

    return 0;
}

#endif //__ICP_ACTION_H__