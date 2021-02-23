/**
 * source=line point1=x1,y1,z1 point2=x2,y2,z2 density=1
 * source=plane origin=x0,y0,z0, point1=x1,y1,z1 point2=x2,y2,z2 density=1
 * source=cube xrange=-1,1 yrange=-2,2 zrange=-3,3 density=2
 * source=circle center=x,y,z radius=1 density=1
 * source=sphere center=x,y,z radius=1 density=1
 * source=noise xrange=-1,1 yrange=-2,2 zrange=-3,3 count=200
 */
#ifndef __SHAPE_ACTION_H__
#define __SHAPE_ACTION_H__

#include <pcl/io/vtk_lib_io.h>

#include <vtkLineSource.h>
#include <vtkPlaneSource.h>
#include <vtkCubeSource.h>
#include <vtkDiskSource.h>
#include <vtkSphereSource.h>
#include <vtkCylinderSource.h>

const float PI = 3.141592653589793;
#define DISTANCE_OF_POINTS(pt1, pt2) \
    sqrt((pt2[0]-pt1[0])*(pt2[0]-pt1[0]) \
        +(pt2[1]-pt1[1])*(pt2[1]-pt1[1]) \
        +(pt2[2]-pt1[2])*(pt2[2]-pt1[2]))

class ShapeAction : public SourceAction
{
    char shape_[16];
    long count_;
    double xmin_, xmax_;
    double ymin_, ymax_;
    double zmin_, zmax_;
    double radius_;
    double origin_[3];
    double point1_[3];
    double point2_[3];
    double center_[3];
    int density_; // points/cm

    vtkSmartPointer<vtkPolyData> createLine();
    vtkSmartPointer<vtkPolyData> createPlane();
    vtkSmartPointer<vtkPolyData> createCircle();
    vtkSmartPointer<vtkPolyData> createSphere();
    void createCube(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);
    void createNoise(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);

public:
    ShapeAction(std::vector<const char*>& actionStr);
    int doit(std::vector<PCW*>& in, std::vector<PCW*>& out);
    void dump() {
        pcl::console::print_highlight("Action: ");
        pcl::console::print_value("%s=%s\n", getClassName().c_str(), getName().c_str());
        pcl::console::print_info("Parameters: ");
        pcl::console::print_value("shape=%s, ", shape_);

        if (!strcasecmp(shape_, "Line")) {
            pcl::console::print_value("point1=(%g,%g,%g), point2=(%g,%g,%g), density=%d\n",
                    point1_[0], point1_[1], point1_[2],
                    point2_[0], point2_[1], point2_[2], density_);
        } else if (!strcasecmp(shape_, "Cube")) {
            pcl::console::print_value("xrange=(%g,%g), yrange=(%g,%g,%g), zrange=(%g,%g,%g), density=%d\n",
                    xmin_, xmax_, ymin_, ymax_, zmin_, zmax_, density_);
        } else if (!strcasecmp(shape_, "Circle")) {
            pcl::console::print_value("center=(%g,%g,%g), radius=%g, density=%d\n",
                    center_[0], center_[1], center_[2], radius_, density_);
        } else if (!strcasecmp(shape_, "Sphere")) {
            pcl::console::print_value("center=(%g,%g,%g), radius=%g, density=%d\n",
                    center_[0], center_[1], center_[2], radius_, density_);
        } else if (!strcasecmp(shape_, "Noise")) {
            pcl::console::print_value("xrange=(%g,%g), yrange=(%g,%g,%g), zrange=(%g,%g,%g), count=%d\n",
                    xmin_, xmax_, ymin_, ymax_, zmin_, zmax_, count_);
        } else { // Plane
            pcl::console::print_value("origin=(%g,%g,%g), point1=(%g,%g,%g), point2=(%g,%g,%g), density=%d\n",
                    origin_[0], origin_[1], origin_[2],
                    point1_[0], point1_[1], point1_[2],
                    point2_[0], point2_[1], point2_[2], density_);
        }
    }
};

ShapeAction::ShapeAction(std::vector<const char*>& actionStr)
:SourceAction("Generate")
{
    xmin_ = ymin_ = zmin_ = -1.0f;
    xmax_ = ymax_ = zmax_ = 1.0f;
    count_ = 100;
    radius_ = 1.0f;
    density_ = 1; // 1 point per cm
    origin_[0] = origin_[1] = origin_[2] = 0;
    center_[0] = center_[1] = center_[2] = 0;
    point1_[0] = 1; point1_[1] = point1_[2] = 0;
    point2_[0] = point2_[2] = 0; point2_[1] = 1;
    strcpy(shape_, "plane");

    for (size_t i=0; i<actionStr.size(); i++)
    {
        char key[64];
        char value[64];
        if (sscanf(actionStr[i], "%[^=]=%s", key, value) == 2) {
            if (!strcasecmp(key, "source")) {
                strncpy(shape_, value, 15);
            } else if (!strcasecmp(key, "xrange")) {
                parse_arguments_2(value, xmin_, xmax_);
            } else if (!strcasecmp(key, "yrange")) {
                parse_arguments_2(value, ymin_, ymax_);
            } else if (!strcasecmp(key, "zrange")) {
                parse_arguments_2(value, zmin_, zmax_);
            } else if (!strcasecmp(key, "count")) {
                count_ = atoi(value);
            } else if (!strcasecmp(key, "center")) {
                parse_arguments_3(value, center_[0], center_[1], center_[2]);
            } else if (!strcasecmp(key, "radius")) {
                radius_ = atof(value);
            } else if (!strcasecmp(key, "origin")) {
                parse_arguments_3(value, origin_[0], origin_[1], origin_[2]);
            } else if (!strcasecmp(key, "point1")) {
                parse_arguments_3(value, point1_[0], point1_[1], point1_[2]);
            } else if (!strcasecmp(key, "point2")) {
                parse_arguments_3(value, point2_[0], point2_[1], point2_[2]);
            } else if (!strcasecmp(key, "density")) {
                density_ = atoi(value);
            }
        }
    }
}

vtkSmartPointer<vtkPolyData> ShapeAction::createLine()
{
    int resolution = DISTANCE_OF_POINTS(point1_, point2_)*(100*density_);

    vtkSmartPointer<vtkLineSource> line = vtkSmartPointer<vtkLineSource>::New();
    line->SetPoint1(point1_); //二个点中的起点
    line->SetPoint2(point2_); //二个点中的终点
    line->SetResolution(resolution);
    line->Update();

    return line->GetOutput();
}

vtkSmartPointer<vtkPolyData> ShapeAction::createPlane()
{
    int resolution1 = DISTANCE_OF_POINTS(origin_, point1_)*(100*density_);
    int resolution2 = DISTANCE_OF_POINTS(origin_, point2_)*(100*density_);

    vtkSmartPointer<vtkPlaneSource> plane = vtkSmartPointer<vtkPlaneSource>::New ();
    plane->SetOrigin(origin_); //三个点中的起点
    plane->SetPoint1(point1_); //三个点中的终点1
    plane->SetPoint2(point2_); //三个点中的终点2
    plane->SetResolution(resolution1, resolution2);
    plane->Update();

    return plane->GetOutput();
}

void ShapeAction::createCube(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
    vtkSmartPointer<vtkCubeSource> cube = vtkSmartPointer<vtkCubeSource>::New();
    cube->SetBounds(xmin_, xmax_, ymin_, ymax_, zmin_, zmax_);
    cube->Update();

    vtkPolyData* polydata = cube->GetOutput();
    for(vtkIdType i = 0; i < polydata->GetNumberOfPoints(); i+=4) {
        /*
        * 生成的cube source共有6个面，每个面用4个点来描述，总共有24个点
        * 我们使用每个面中的3个点并调用createPlane来生成一个平面
        */
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPlane(new pcl::PointCloud<pcl::PointXYZ>);
        polydata->GetPoint(i, origin_);
        polydata->GetPoint(i+1, point1_);
        polydata->GetPoint(i+2, point2_);
        pcl::io::vtkPolyDataToPointCloud(createPlane(), *cloudPlane);
        *cloud += *cloudPlane;
    }
}

vtkSmartPointer<vtkPolyData> ShapeAction::createCircle()
{
    int resolution = (2*PI*radius_)*(100*density_);

    vtkSmartPointer<vtkDiskSource> disk = vtkSmartPointer<vtkDiskSource>::New ();
    disk->SetInnerRadius(0);
    disk->SetOuterRadius(radius_);
    disk->SetCircumferentialResolution(resolution);
    disk->SetRadialResolution(resolution);
    disk->Update();

    // fixed the center of circle
    vtkPoints* points = disk->GetOutput()->GetPoints();
    for(vtkIdType i = 0; i < points->GetNumberOfPoints(); i++) {
        double pt[3];
        points->GetPoint(i, pt);
        pt[0] += center_[0];
        pt[1] += center_[1];
        pt[2] += center_[2];
        points->SetPoint(i, pt);
    }

    return disk->GetOutput();
}

vtkSmartPointer<vtkPolyData> ShapeAction::createSphere()
{
    int resolution = (2*PI*radius_)*(100*density_);

    vtkSmartPointer<vtkSphereSource> sphere = vtkSmartPointer<vtkSphereSource>::New();
    sphere->SetRadius(radius_);
    sphere->SetCenter(center_);
    sphere->SetThetaResolution(resolution);
    sphere->SetPhiResolution(resolution);
    sphere->Update();

    return sphere->GetOutput();
}

void ShapeAction::createNoise(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
    // 需要参数: x/y/z range, 噪点数量
    cloud->width = count_;
    cloud->height = 1;
    cloud->points.resize (cloud->width * cloud->height);

    for (long i=0; i<count_; i++) {
        cloud->points[i].x = xmin_ + (xmax_-xmin_) * rand() / (RAND_MAX + 1.0f);
        cloud->points[i].y = ymin_ + (ymax_-ymin_) * rand() / (RAND_MAX + 1.0f);
        cloud->points[i].z = zmin_ + (zmax_-zmin_) * rand() / (RAND_MAX + 1.0f);
    }
}

int ShapeAction::doit(std::vector<PCW*>& in, std::vector<PCW*>& out)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    vtkSmartPointer<vtkPolyData> polydata = NULL;

    if (!strcasecmp(shape_, "Line")) {
        polydata = createLine();
    } else if (!strcasecmp(shape_, "Plane")) {
        polydata = createPlane();
    } else if (!strcasecmp(shape_, "Cube")) {
        createCube(cloud);
    } else if (!strcasecmp(shape_, "Circle")) {
        polydata = createCircle();
    } else if (!strcasecmp(shape_, "Sphere")) {
        polydata = createSphere();
    } else if (!strcasecmp(shape_, "Noise")) {
        createNoise(cloud);
    } else {
        polydata = createPlane();
    }

    if (polydata != NULL)
        pcl::io::vtkPolyDataToPointCloud(polydata, *cloud);

    pcl::toPCLPointCloud2(*cloud, *(out[0]->cloud));

    return 0;
}

#endif //__SHAPE_ACTION_H__