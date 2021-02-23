/**
 * filter=voxelgrid leaf=0.01
 **/

#ifndef __VOXEL_GRID_ACTION_H__
#define __VOXEL_GRID_ACTION_H__

#include <pcl/filters/voxel_grid.h>

class VoxelGridAction : public FilterAction
{
    float leaf_;

public:
    VoxelGridAction(std::vector<const char*>& vector);
    int doit(std::vector<PCW*>& in, std::vector<PCW*>& out);
    void dump() {
        pcl::console::print_highlight("Action: ");
        pcl::console::print_value("%s=%s\n", getClassName().c_str(), getName().c_str());
        pcl::console::print_info("Parameters: ");
        pcl::console::print_value("leaf=%g\n", leaf_);
    }
};

VoxelGridAction::VoxelGridAction(std::vector<const char*>& actionStr)
:FilterAction("VoxelGrid")
{
    leaf_ = 0.01;

    for (size_t i=0; i<actionStr.size(); i++) {
        char key[64];
        char value[64];
        if (sscanf(actionStr[i], "%[^=]=%s", key, value) == 2) {
            if (!strcasecmp(key, "leaf")) {
                leaf_ = atof(value);
            }
        }
    }
}

int VoxelGridAction::doit(std::vector<PCW*>& in, std::vector<PCW*>& out)
{
    pcl::VoxelGrid<pcl::PCLPointCloud2> vg;
    vg.setInputCloud (in[0]->cloud);
    vg.setLeafSize (leaf_, leaf_, leaf_);
    vg.filter (*(out[0]->cloud));

    // 输入点云已经没有用，可释放掉
    in[0]->clear();

    return 0;
}

#endif //__VOXEL_GRID_ACTION_H__