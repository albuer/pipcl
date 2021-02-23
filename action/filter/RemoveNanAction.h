/**
 * filter=removeNan
 **/

#ifndef __REMOVE_NAN_ACTION_H__
#define __REMOVE_NAN_ACTION_H__

class RemoveNanAction : public FilterAction
{
    template <typename PointT>
    int doitAsType(const PointCloud2 &cloud2_in, PointCloud2& cloud2_out);
public:
    RemoveNanAction(std::vector<const char*>& vector):FilterAction("RemoveNaN") {}
    int doit(std::vector<PCW*>& in, std::vector<PCW*>& out);
    void dump() {
        pcl::console::print_highlight("Action: ");
        pcl::console::print_value("%s=%s\n", getClassName().c_str(), getName().c_str());
    }
};

template <typename PointT>
int RemoveNanAction::doitAsType(const PointCloud2 &cloud2_in, PointCloud2& cloud2_out)
{
    typename pcl::PointCloud<PointT>::Ptr cloud_in (new pcl::PointCloud<PointT>);
    typename pcl::PointCloud<PointT>::Ptr cloud_out (new pcl::PointCloud<PointT>);
    pcl::fromPCLPointCloud2(cloud2_in, *cloud_in);

    std::vector<int> indices;
    pcl::removeNaNFromPointCloud<PointT> (*cloud_in, *cloud_out, indices);

    toPCLPointCloud2(*cloud_out, cloud2_out);

    return 0;
}

int RemoveNanAction::doit(std::vector<PCW*>& in, std::vector<PCW*>& out)
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

#endif //__REMOVE_NAN_ACTION_H__