/**
 * sink=xxx.pcd ascii=1
 **/

#ifndef __FILEWRITER_ACTION_H__
#define __FILEWRITER_ACTION_H__

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/ifs_io.h>

class FileWriterAction : public SinkAction
{
    bool ascii_;
    std::string filename_;
public:
    FileWriterAction(std::string filename):SinkAction("FileWriter") {
        filename_ = filename;
        ascii_ = false;
    }
    FileWriterAction(std::vector<const char*>& vector);
    int doit(std::vector<PCW*>& in, std::vector<PCW*>& out);
    void dump() {
        pcl::console::print_highlight("Action: ");
        pcl::console::print_value("%s=%s\n", getClassName().c_str(), getName().c_str());
        pcl::console::print_info("Parameters: ");
        pcl::console::print_value("file=%s, ascii=%d\n", filename_.c_str(), ascii_);
    }
};

FileWriterAction::FileWriterAction(std::vector<const char*>& actionStr)
:SinkAction("FileWriter")
{
    ascii_ = false;

    for (size_t i=0; i<actionStr.size(); i++) {
        char key[64];
        char value[256];
        if (sscanf(actionStr[i], "%[^=]=%s", key, value) == 2) {
            if (!strcasecmp(key, "sink")) {
                filename_ = value;
            } else if (!strcasecmp(key, "ascii")) {
                ascii_ = !!atoi(value);
            }
        }
    }
}

int FileWriterAction::doit(std::vector<PCW*>& in, std::vector<PCW*>& out) {
    // 根据文件扩展名自动调用相应的加载函数
    // 只支持 pcd/ply/ifs 三种格式
    boost::filesystem::path p(filename_.c_str());
    std::string extension = p.extension().string();
    int result = -1;
    Eigen::Vector4f origin = Eigen::Vector4f::Zero ();
    Eigen::Quaternionf orientation = Eigen::Quaternionf::Identity ();

    if (extension == ".pcd")
        result = pcl::io::savePCDFile(filename_, *(in[0]->cloud), origin, orientation, !ascii_);
    else if (extension == ".ply")
        result = pcl::io::savePLYFile(filename_, *(in[0]->cloud), origin, orientation, !ascii_);
    else if (extension == ".ifs")
        result = pcl::io::saveIFSFile(filename_, *(in[0]->cloud));
    else {
        PCL_ERROR("[pcl::io::save] Don't know how to handle file with extension %s\n",
                extension.c_str());
        result = -1;
    }

    if (!out.empty()) {
        in[0]->moveTo(out[0]);
    } else {
        // 输入点云已经没有用，可释放掉
        in[0]->clear();
    }

    return (result);
}

#endif //__FILEWRITER_ACTION_H__