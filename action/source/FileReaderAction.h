#ifndef __FILEREADER_ACTION_H__
#define __FILEREADER_ACTION_H__

#include <pcl/io/auto_io.h>

class FileReaderAction : public SourceAction
{
    std::string filename_;
public:
    FileReaderAction(std::string filename):SourceAction("FileRead"),filename_(filename) { }

    // 从指定文件读取数据，并保存在点云变量中
    int doit(std::vector<PCW*>& in, std::vector<PCW*>& out);
    void dump() {
        pcl::console::print_highlight("Action: ");
        pcl::console::print_value("%s=%s\n", getClassName().c_str(), getName().c_str());
        pcl::console::print_info("Parameters: ");
        pcl::console::print_value("file=%s\n", filename_.c_str());
    }
};

int FileReaderAction::doit(std::vector<PCW*>& in, std::vector<PCW*>& out)
{
    // 根据文件扩展名自动调用相应的加载函数
    // 只支持 pcd/ply/ifs 三种格式

    pcl::io::load(filename_, *(out[0]->cloud));

    return 0;
}

// 支持vtk文件　tools/vtk2pcd.cpp
// 支持obj文件  tools/obj2pcd.cpp
// 支持xyz文件  tools/xyz2pcd.cpp

#endif //__FILEREADER_ACTION_H__