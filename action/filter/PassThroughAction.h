/**
 * filter=passthrough field=x min=-0.05 max=0.05 negative=0
 **/

#ifndef __PASSTHROUGH_ACTION_H__
#define __PASSTHROUGH_ACTION_H__

#include <pcl/filters/passthrough.h>

class PassThroughAction : public FilterAction
{
    std::string field_name_;
    float min_;
    float max_;
    bool negative_;

public:
    PassThroughAction(std::string field_name, float min, float max, bool negative=false);
    PassThroughAction(std::vector<const char*>& vector);
    int doit(std::vector<PCW*>& in, std::vector<PCW*>& out);
    void dump() {
        pcl::console::print_highlight("Action: ");
        pcl::console::print_value("%s=%s\n", getClassName().c_str(), getName().c_str());
        pcl::console::print_info("Parameters: ");
        pcl::console::print_value("field=%s, range=(%g,%g), negative=%d\n",
                field_name_.c_str(), min_, max_, negative_);
    }
};

PassThroughAction::PassThroughAction(std::string field_name, float min, float max, bool negative)
:FilterAction("PassThrough")
{
    field_name_ = field_name;
    min_ = min;
    max_ = max;
    negative_ = negative;
}

PassThroughAction::PassThroughAction(std::vector<const char*>& actionStr)
:FilterAction("PassThrough")
{
    field_name_ = "x";
    min_ = 0.0f;
    max_ = 1.0f;
    negative_ = false;

    for (size_t i=0; i<actionStr.size(); i++) {
        char key[64];
        char value[64];
        if (sscanf(actionStr[i], "%[^=]=%s", key, value) == 2) {
            if (!strcasecmp(key, "field")) {
                field_name_ = value;
            } else if (!strcasecmp(key, "min")) {
                min_ = atof(value);
            } else if (!strcasecmp(key, "max")) {
                max_ = atof(value);
            } else if (!strcasecmp(key, "negative")) {
                negative_ = !!(atoi(value));
            }
        }
    }
}

int PassThroughAction::doit(std::vector<PCW*>& in, std::vector<PCW*>& out) {

    pcl::PassThrough<pcl::PCLPointCloud2> passthrough_filter;
    PCW* pcw_in = in[0];
    passthrough_filter.setInputCloud (in[0]->cloud);
    passthrough_filter.setFilterFieldName (field_name_);
    passthrough_filter.setFilterLimits (min_, max_);

    passthrough_filter.setNegative (negative_);
    passthrough_filter.filter (*(out[0]->cloud));

    if (out[1] != NULL) {
        // 输出反向选择的点云
        passthrough_filter.setNegative (!negative_);
        passthrough_filter.filter (*(out[1]->cloud));
    }

    // 输入点云已经没有用，可释放掉
    in[0]->clear();

    return 0;
}

#endif //__PASSTHROUGH_ACTION_H__