#ifndef __ACTION_H__
#define __ACTION_H__

#include <string>
#include <vector>

#include <pcl/PCLPointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/console/parse.h>
#include <pcl/console/print.h>

typedef pcl::PCLPointCloud2             PointCloud2;
typedef PointCloud2::Ptr                PointCloud2Ptr;

struct PCW {
    PointCloud2Ptr  cloud;
    std::string     name;
    void*           reserve;

    PCW() : cloud(new PointCloud2) {}

    void clear() {
        cloud = NULL;
        name = "";
    }

    void moveTo(PCW* pcw) {
        pcw->clear();
        pcw->cloud = this->cloud;
        pcw->name = this->name;
        this->clear();
    }
};

void parseCloudField(const PointCloud2Ptr cloud, bool& has_rgb, bool& has_normals)
{
    has_rgb = false;
    has_normals = false;
    for (size_t i = 0; i < cloud->fields.size(); i++)
    {
        if (cloud->fields[i].name.find("rgb") != std::string::npos)
            has_rgb = true;
        if (cloud->fields[i].name == "normal_x")
            has_normals = true;
    }
}

int parse_arguments_2(const char* arg, double& x1, double& x2)
{
    int ret = -1;
    char str1[32] = "";
    char str2[32] = "";
    ret = sscanf(arg, "%[^,],%s", str1, str2);
    if (ret > 0) {
        if (str1[0]!='\0') x1 = atof(str1);
        if (str2[0]!='\0') x2 = atof(str2);
    }

    return ret;
}

int parse_arguments_3(const char* arg, double& x1, double& x2, double& x3)
{
    int ret = -1;
    char str1[32] = "";
    char str2[32] = "";
    char str3[32] = "";
    ret = sscanf(arg, "%[^,],%[^,],%s", str1, str2, str3);
    if (ret > 0) {
        if (str1[0]!='\0') x1 = atof(str1);
        if (str2[0]!='\0') x2 = atof(str2);
        if (str3[0]!='\0') x3 = atof(str3);
    }

    return ret;
}

int parse_arguments_x(const char* arg, std::vector<double>& v)
{
    int ret = 0;
    char str[256] = "";
    float x = 0.0f;
    do {
        ret = sscanf(arg, "%f,%s", &x, str);
        v.push_back((double)x);
        arg = str;
    } while (ret >= 2);

    return ret;
}

class Action
{
public:
    enum _Class_ {
        MISC=0,
        SOURCE,
        SINK,
        FILTER,
        SEARCH,
        SEGMENTATION,
        FEATURE,
        REGISTRATION,

        MAX_CLASS
    };

private:
    // action名称
    std::string name_;
    // action类型
    int class_;
    // 指示当前这个类型的action已经实例化了多少个
    int index_;
    // 各种action类型的已实例化的数量
    static int instance_[MAX_CLASS];
    static std::string class_name_[MAX_CLASS];

public:
    Action(std::string n, int Class=Action::MISC) {
        name_ = n;
        class_ = Class;
        index_ = instance_[Class];
        instance_[Class]++;
    }
    std::string getName() { return name_; }
    std::string getClassName() { return class_name_[class_]; }
    int getClass() { return class_; }
    static std::vector<Action*> parse(std::vector<const char*>& vector, int batch);
    static void showHelp(const char* actionClass);
    virtual int doit(std::vector<PCW*>& in, std::vector<PCW*>& out) = 0;
    virtual void dump() {
        pcl::console::print_highlight("Action: ");
        pcl::console::print_value("%s=%s\n", getClassName().c_str(), getName().c_str());
    }
};

int Action::instance_[] = {0};
std::string Action::class_name_[] = {
    "MISC",
    "Source",
    "Sink",
    "Filter",
    "Search",
    "Segmentation",
    "Feature",
    "Registration",
};

inline std::ostream& operator<<(std::ostream& file, Action& action)
{
    return file << action.getName();
}

inline std::ostream& operator<<(std::ostream& file, Action* action)
{
    return file << (action?action->getName():"null");
}

#include "source/SourceAction.h"
#include "sink/SinkAction.h"
#include "filter/FilterAction.h"
#include "search/SearchAction.h"
#include "segmentation/SegmentAction.h"
#include "misc/MiscAction.h"
#include "feature/FeatureAction.h"
#include "registration/RegistrationAction.h"

std::vector<Action*> Action::parse(std::vector<const char*>& actionStr, int batch)
{
    std::vector<Action*> actionList;
    const char* actionCmd = actionStr.front();

    if (!strncasecmp(actionCmd, "source=", 7)) {
        actionList = SourceAction::parse(actionStr);
    } else if (!strncasecmp(actionCmd, "sink=", 5)) {
        actionList = SinkAction::parse(actionStr, batch);
    } else if (!strncasecmp(actionCmd, "filter=", 7)) {
        actionList = FilterAction::parse(actionStr, batch);
    } else if (!strncasecmp(actionCmd, "search=", 7)) {
        actionList = SearchAction::parse(actionStr, batch);
    } else if (!strncasecmp(actionCmd, "misc=", 5)) {
        actionList = MiscAction::parse(actionStr, batch);
    } else if (!strncasecmp(actionCmd, "segmentation=", 13)) {
        actionList = SegmentAction::parse(actionStr, batch);
    } else if (!strncasecmp(actionCmd, "feature=", 8)) {
        actionList = FeatureAction::parse(actionStr, batch);
    } else if (!strncasecmp(actionCmd, "registration=", 13)) {
        actionList = RegistrationAction::parse(actionStr, batch);
    } else {
        std::cerr << "Invalid action: " << actionCmd << std::endl;
    }

    return actionList;
}

void Action::showHelp(const char* actionClass)
{
    if (!strcasecmp(actionClass, "source")) {
        SourceAction::showHelp();
    } else if (!strcasecmp(actionClass, "sink")) {
        SinkAction::showHelp();
    } else if (!strcasecmp(actionClass, "filter")) {
        FilterAction::showHelp();
    } else if (!strcasecmp(actionClass, "search")) {
        SearchAction::showHelp();
    } else if (!strcasecmp(actionClass, "misc")) {
        MiscAction::showHelp();
    } else if (!strcasecmp(actionClass, "segmentation")) {
        SegmentAction::showHelp();
    } else if (!strcasecmp(actionClass, "feature")) {
        FeatureAction::showHelp();
    } else if (!strcasecmp(actionClass, "registration")) {
        RegistrationAction::showHelp();
    } else {
        std::cerr << "Invalid action class: " << actionClass << std::endl;
    }
}

#endif //__ACTION_H__