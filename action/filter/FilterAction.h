#ifndef __FILTER_ACTION_H__
#define __FILTER_ACTION_H__

class FilterAction : public Action
{
public:
    FilterAction(std::string n):Action(n, Action::FILTER) {}
    static std::vector<Action*> parse(std::vector<const char*>& actionStr, int batch);
    static void showHelp(void);
};

#include "PassThroughAction.h"
#include "VoxelGridAction.h"
#include "SorAction.h"
#include "RorAction.h"
#include "RemoveNanAction.h"

std::vector<Action*> FilterAction::parse(std::vector<const char*>& actionStr, int batch) {
    std::vector<Action*> actionList;
    const char* filter = actionStr.front()+strlen("filter=");

    // todo: 判断filter类型
    FilterAction* action;
    for (int i=0; i<batch; i++) {
        if (!strcasecmp(filter, "passthrough")) {
            action = new PassThroughAction(actionStr);
        } else if (!strcasecmp(filter, "VoxelGrid")) {
            action = new VoxelGridAction(actionStr);
        } else if (!strcasecmp(filter, "SOR")) {
            action = new SorAction(actionStr);
        } else if (!strcasecmp(filter, "ROR")) {
            action = new RorAction(actionStr);
        } else if (!strcasecmp(filter, "RemoveNaN")) {
            action = new RemoveNanAction(actionStr);
        } else {
            action = NULL;
        }

        if (action != NULL)
            actionList.push_back(action);
    }

    return actionList;
}

void FilterAction::showHelp(void)
{
    printf("\n");
    printf("filter=<ACTION> [ARGS]\n");
    printf("\n");
    printf("All filter actions are:\n");
    printf("\n");
    printf("  Passthrough        Passthrough Filter\n");
    printf("    field=x            The name of the field (x|y|z) that will be used for filtering\n");
    printf("    min=0.0            The minimum allowed field value\n");
    printf("    max=1.0            The maximum allowed field value\n");
    printf("    negative=0         Inside the interval (0) or outside (1)\n");
    printf("\n");
    printf("  VoxelGrid          Voxel Grid Filter\n");
    printf("    leaf=0.01          the voxel grid leaf size\n");
    printf("\n");
    printf("  SOR                Statistical Outlier Removal\n");
    printf("    meank=2            The number of points to use for mean distance estimation\n");
    printf("    stdmul=0.0         The standard deviation multiplier\n");
    printf("\n");
    printf("  ROR                Radius Outlier Removal\n");
    printf("    minpts=2           the minimum number of neighbors\n");
    printf("    radius=0.8         The radius of the sphere for nearest neighbor searching\n");
    printf("\n");
    printf("  RemoveNaN          Invalid Points Removal\n");
    printf("\n");
}

#endif //__FILTER_ACTION_H__