#ifndef __SEGMENT_ACTION_H__
#define __SEGMENT_ACTION_H__

class SegmentAction : public Action
{
public:
    SegmentAction(std::string n):Action(n, Action::SEGMENTATION) {}
    static std::vector<Action*> parse(std::vector<const char*>& actionStr, int batch);
    static void showHelp(void);
};

#include "SacAction.h"

std::vector<Action*> SegmentAction::parse(std::vector<const char*>& actionStr, int batch) {
    std::vector<Action*> actionList;
    const char* filter = actionStr.front()+strlen("segmentation=");

    SegmentAction* action;
    for (int i=0; i<batch; i++) {
        if (!strcasecmp(filter, "SAC")) {
            action = new SacAction(actionStr);
        } else {
            action = NULL;
        }

        if (action != NULL)
            actionList.push_back(action);
    }

    return actionList;
}

void SegmentAction::showHelp(void)
{
    printf("\n");
    printf("segmentation=<ACTION> [ARGS]\n");
    printf("\n");
    printf("All segmentation actions are:\n");
    printf("\n");
    printf("  SAC                Sample Consensus\n");
    printf("    model=PLANE        the model type (PLANE|LINE|CIRCLE2D|CIRCLE3D|SPHERE|CYLINDER|CONE|TORUS)\n");
    printf("    method=RANSAC      the method type (RANSAC|LMEDS|MSAC|RRANSAC|RMSAC|MLESAC|PROSAC)\n");
    printf("    iterations=50      the maximum number of iterations the sample consensus method will run\n");
    printf("    threshold=0.1      the distance threshold to use\n");
    printf("\n");
}

#endif //__SEGMENT_ACTION_H__