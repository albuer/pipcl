#ifndef __MIX_ACTION_H__
#define __MIX_ACTION_H__

/**
 * 点云合并，以第一个点云为基准，合并后续的点云
 *   field合并时，取第一个点云的points数据，然后合入后续点云的其它field；最终输出的points不变，field增加
 *   points合并时候，以第一个点云的filed为准，合并后续点云的对应field下points数据；最终输出的field不变，points增加
 **/
class MixAction : public MiscAction
{
public:
    MixAction():MiscAction("Mix") {};
    int doit(std::vector<PCW*>& in, std::vector<PCW*>& out);
};

template <typename PointT>
void addPointCloud2AsType(const PointCloud2 &input, PointCloud2 &output)
{
    pcl::PointCloud<PointT> cloud_in, cloud_out;
    fromPCLPointCloud2(input, cloud_in);
    fromPCLPointCloud2(output, cloud_out);
    cloud_out += cloud_in;
    toPCLPointCloud2(cloud_out, output);
}

int MixAction::doit(std::vector<PCW*>& in, std::vector<PCW*>& out)
{
    in[0]->moveTo(out[0]);

    bool out_has_rgb, out_has_normals;
    parseCloudField(out[0]->cloud, out_has_rgb, out_has_normals);

    for (size_t i=1; i<in.size(); i++) {
        // Check for 'rgb' and 'normals' fields
        bool in_has_rgb, in_has_normals;
        PointCloud2Ptr cloud = in[i]->cloud;
        parseCloudField(cloud, in_has_rgb, in_has_normals);

        if ((out_has_rgb != in_has_rgb || out_has_normals != in_has_normals) &&
            out[0]->cloud->height == cloud->height && out[0]->cloud->width == cloud->width) {
            // field不同，且points相同时，field合并
            PointCloud2 out_cloud;
            concatenateFields (*cloud, *(out[0]->cloud), out_cloud);
            *(out[0]->cloud) = out_cloud;
        } else {
            // 进行点合并
            if (!out_has_rgb && !out_has_normals)
                addPointCloud2AsType<pcl::PointXYZ> (*cloud, *(out[0]->cloud));
            else if (out_has_rgb && !out_has_normals)
                addPointCloud2AsType<pcl::PointXYZRGB> (*cloud, *(out[0]->cloud));
            else  if (!out_has_rgb && out_has_normals)
                addPointCloud2AsType<pcl::PointNormal> (*cloud, *(out[0]->cloud));
            else
                addPointCloud2AsType<pcl::PointXYZRGBNormal> (*cloud, *(out[0]->cloud));
        }

        // 输入点云已经没有用，可释放掉
        in[i]->clear();
    }

    return 0;
}

#endif //__MIX_ACTION_H__