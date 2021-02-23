#ifndef __VISUALIZATION_ACTION_H__
#define __VISUALIZATION_ACTION_H__

#include <iostream>
#include <thread>

#include <pcl/visualization/pcl_visualizer.h>

using namespace std::chrono_literals;

class VisualizationAction : public SinkAction
{
    std::string viewName_;
    template <typename PointT>
    int doitAsType(const PointCloud2 &cloud);
public:
    VisualizationAction(std::string n="Vis"):SinkAction("Visualization") { viewName_ = n; }
    int doit(std::vector<PCW*>& in, std::vector<PCW*>& out) {
        bool has_rgb, has_normals;
        parseCloudField(in[0]->cloud, has_rgb, has_normals);

        int ret = 0;
        // 根据具体点云数据类型进行处理
        if (!has_rgb && !has_normals)
            ret = doitAsType<pcl::PointXYZ> (*(in[0]->cloud));
        else if (has_rgb && !has_normals)
            ret = doitAsType<pcl::PointXYZRGB> (*(in[0]->cloud));
        else  if (!has_rgb && has_normals)
            ret = doitAsType<pcl::PointNormal> (*(in[0]->cloud));
        else
            ret = doitAsType<pcl::PointXYZRGBNormal> (*(in[0]->cloud));

        return ret;
    }
};

template <typename PointT>
int VisualizationAction::doitAsType(const PointCloud2 &cloud_in)
{
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer (viewName_));
    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT> ());
    pcl::fromPCLPointCloud2(cloud_in, *cloud);
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addPointCloud<PointT> (cloud, "VIS");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "VIS");
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        std::this_thread::sleep_for(100ms);
    }

    return 0;
}

#endif //__VISUALIZATION_ACTION_H__