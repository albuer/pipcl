// misc=transform trans=dx,dy,dz quat=x,y,z,w axis= scale= matrix= 
#ifndef __TRANSFORM_ACTION_H__
#define __TRANSFORM_ACTION_H__

#include <pcl/common/transforms.h>

class TransformAction : public MiscAction
{
    Eigen::Matrix4f tform_;
    double scale_[3];

public:
    TransformAction(std::vector<const char*>& actionStr);
    int doit(std::vector<PCW*>& in, std::vector<PCW*>& out);
    void dump();
};

template <typename T>
void multiply(pcl::PCLPointCloud2 &cloud, int field_offset, double multiplier)
{
    T val;
    memcpy(&val, &cloud.data[field_offset], sizeof(T));
    val = static_cast<T>(val * static_cast<T>(multiplier));
    memcpy(&cloud.data[field_offset], &val, sizeof(T));
}

void scaleInPlace(pcl::PCLPointCloud2 &cloud, double *multiplier)
{
    // Obtain the x, y, and z indices
    int x_idx = pcl::getFieldIndex(cloud, "x");
    int y_idx = pcl::getFieldIndex(cloud, "y");
    int z_idx = pcl::getFieldIndex(cloud, "z");
    Eigen::Array3i xyz_offset(cloud.fields[x_idx].offset, cloud.fields[y_idx].offset, cloud.fields[z_idx].offset);

    for (uint32_t cp = 0; cp < cloud.width * cloud.height; ++cp)
    {
        // Assume all 3 fields are the same (XYZ)
        assert((cloud.fields[x_idx].datatype == cloud.fields[y_idx].datatype));
        assert((cloud.fields[x_idx].datatype == cloud.fields[z_idx].datatype));
        switch (cloud.fields[x_idx].datatype)
        {
        case pcl::PCLPointField::INT8:
            for (int i = 0; i < 3; ++i)
                multiply<int8_t>(cloud, xyz_offset[i], multiplier[i]);
            break;
        case pcl::PCLPointField::UINT8:
            for (int i = 0; i < 3; ++i)
                multiply<uint8_t>(cloud, xyz_offset[i], multiplier[i]);
            break;
        case pcl::PCLPointField::INT16:
            for (int i = 0; i < 3; ++i)
                multiply<int16_t>(cloud, xyz_offset[i], multiplier[i]);
            break;
        case pcl::PCLPointField::UINT16:
            for (int i = 0; i < 3; ++i)
                multiply<uint16_t>(cloud, xyz_offset[i], multiplier[i]);
            break;
        case pcl::PCLPointField::INT32:
            for (int i = 0; i < 3; ++i)
                multiply<int32_t>(cloud, xyz_offset[i], multiplier[i]);
            break;
        case pcl::PCLPointField::UINT32:
            for (int i = 0; i < 3; ++i)
                multiply<uint32_t>(cloud, xyz_offset[i], multiplier[i]);
            break;
        case pcl::PCLPointField::FLOAT32:
            for (int i = 0; i < 3; ++i)
                multiply<float>(cloud, xyz_offset[i], multiplier[i]);
            break;
        case pcl::PCLPointField::FLOAT64:
            for (int i = 0; i < 3; ++i)
                multiply<double>(cloud, xyz_offset[i], multiplier[i]);
            break;
        }
        xyz_offset += cloud.point_step;
    }
}

void TransformAction::dump()
{
    std::stringstream ss;

    // 从变换矩阵中取出旋转矩阵
    Eigen::Matrix3f matrix = tform_.topLeftCorner(3, 3);

    // 旋转矩阵　转　四元数
    Eigen::Quaternionf quaternion(matrix);

    // 旋转矩阵 转 旋转向量
    Eigen::AngleAxisf rotation_vector(matrix);

    // 旋转矩阵 转 欧拉角
    Eigen::Vector3f eulerAngle = matrix.eulerAngles(2, 1, 0); // ZYX顺序，yaw,pitch,roll

    ss << "Transformation matrix:\n"
       << tform_ << "\n"
       << std::endl;
    ss << "Inverse matrix:\n"
       << tform_.inverse() << "\n"
       << std::endl;
    ss << "Transpose matrix:\n"
       << tform_.transpose() << "\n"
       << std::endl;
    ss << "Translation vector(tx,ty,tz):\n"
       << tform_(0, 3) << ","
       << tform_(1, 3) << ","
       << tform_(2, 3) << "\n"
       << std::endl;
    ss << "Rotation vector(ax,ay,az,theta):\n"
       << rotation_vector.axis()(0) << ","
       << rotation_vector.axis()(1) << ","
       << rotation_vector.axis()(2) << ","
       << rotation_vector.angle() << "\n"
       << std::endl;
    ss << "Quaternion(qx,qy,qz,qw):\n"
       << quaternion.x() << ","
       << quaternion.y() << ","
       << quaternion.z() << ","
       << quaternion.w() << "\n"
       << std::endl;
    ss << "Euler angle(yaw,pitch,roll):\n"
       << eulerAngle(0) << ","
       << eulerAngle(1) << ","
       << eulerAngle(2) << "\n"
       << std::endl;

    pcl::console::print_highlight("Action: ");
    pcl::console::print_value("%s=%s\n", getClassName().c_str(), getName().c_str());
    pcl::console::print_info("Parameters: ");
    pcl::console::print_value("\n%s", ss.str().c_str());
}

TransformAction::TransformAction(std::vector<const char*>& actionStr)
:MiscAction("Transfrom")
{
    tform_.setIdentity ();
    scale_[0] = scale_[1] = scale_[2] = 1.0f;

    for (size_t i=0; i<actionStr.size(); i++) {
        char key[64];
        char value[256];
        if (sscanf(actionStr[i], "%[^=]=%s", key, value) == 2) {
            if (!strcasecmp(key, "trans")) {
                double dx, dy, dz;
                parse_arguments_3(value, dx, dy, dz);
                tform_ (0, 3) = dx;
                tform_ (1, 3) = dy;
                tform_ (2, 3) = dz;
            } else if (!strcasecmp(key, "quat")) {
                double x,y,z,w;
                std::vector<double> values;
                parse_arguments_x(value, values);
                x = values[0]; y = values[1]; z = values[2]; w = values[3];
                tform_.topLeftCorner (3, 3) = Eigen::Matrix3f (Eigen::Quaternionf (w, x, y, z));
            } else if (!strcasecmp(key, "axis")) {
                double ax, ay, az, theta;
                std::vector<double> values;
                parse_arguments_x(value, values);
                ax = values[0]; ay = values[1]; az = values[2]; theta = values[3];
                tform_.topLeftCorner (3, 3) = Eigen::Matrix3f (Eigen::AngleAxisf (theta, Eigen::Vector3f (ax, ay, az)));
            } else if (!strcasecmp(key, "matrix")) {
                std::vector<double> values;
                parse_arguments_x(value, values);
                int n = values.size () == 9 ? 3 : 4;
                for (int r = 0; r < n; ++r)
                    for (int c = 0; c < n; ++c)
                        tform_ (r, c) = values[n*r+c];
            } else if (!strcasecmp(key, "scale")) {
                parse_arguments_3(value, scale_[0], scale_[1], scale_[2]);
            }
        }
    }
}

template <typename PointT>
void transformPointCloudHelper(pcl::PointCloud<PointT> &input,
                               pcl::PointCloud<PointT> &output,
                               Eigen::Matrix4f &tform)
{
    transformPointCloud(input, output, tform);
}

template <>
void transformPointCloudHelper(pcl::PointCloud<pcl::PointNormal> &input,
                               pcl::PointCloud<pcl::PointNormal> &output,
                               Eigen::Matrix4f &tform)
{
    transformPointCloudWithNormals(input, output, tform);
}

template <>
void transformPointCloudHelper<pcl::PointXYZRGBNormal>(pcl::PointCloud<pcl::PointXYZRGBNormal> &input,
                                                  pcl::PointCloud<pcl::PointXYZRGBNormal> &output,
                                                  Eigen::Matrix4f &tform)
{
    transformPointCloudWithNormals(input, output, tform);
}

template <typename PointT>
void transformPointCloud2AsType(const PointCloud2 &input, PointCloud2 &output,
                                Eigen::Matrix4f &tform)
{
    pcl::PointCloud<PointT> cloud;
    fromPCLPointCloud2(input, cloud);
    transformPointCloudHelper(cloud, cloud, tform);
    toPCLPointCloud2(cloud, output);
}

int TransformAction::doit(std::vector<PCW*>& in, std::vector<PCW*>& out)
{
    bool has_rgb, has_normals;
    parseCloudField(in[0]->cloud, has_rgb, has_normals);

    if (!has_rgb && !has_normals)
        transformPointCloud2AsType<pcl::PointXYZ>(*(in[0]->cloud), *(out[0]->cloud), tform_);
    else if (has_rgb && !has_normals)
        transformPointCloud2AsType<pcl::PointXYZRGB>(*(in[0]->cloud), *(out[0]->cloud), tform_);
    else if (!has_rgb && has_normals)
        transformPointCloud2AsType<pcl::PointNormal>(*(in[0]->cloud), *(out[0]->cloud), tform_);
    else // (has_rgb && has_normals)
        transformPointCloud2AsType<pcl::PointXYZRGBNormal>(*(in[0]->cloud), *(out[0]->cloud), tform_);

    scaleInPlace (*(out[0]->cloud), scale_);

    // 输入点云已经没有用，可释放掉
    in[0]->clear();

    return 0;
}

#endif //__TRANSFORM_ACTION_H__