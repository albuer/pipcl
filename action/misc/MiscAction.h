#ifndef __MISC_ACTION_H__
#define __MISC_ACTION_H__

class MiscAction : public Action
{
public:
    MiscAction(std::string n):Action(n, Action::MISC) {}
    static std::vector<Action*> parse(std::vector<const char*>& actionStr, int batch);
    static void showHelp(void);
};

#include "MixAction.h"
#include "TransformAction.h"

/**
 * copy
 **/
class CopyAction : public MiscAction
{
public:
    CopyAction():MiscAction("Copy") {};
    int doit(std::vector<PCW*>& in, std::vector<PCW*>& out);
};

int CopyAction::doit(std::vector<PCW*>& in, std::vector<PCW*>& out)
{
    in[0]->moveTo(out[0]);
    return 0;
}

std::vector<Action*> MiscAction::parse(std::vector<const char*>& actionStr, int batch) {
    std::vector<Action*> actionList;
    const char* actionCmd = actionStr.front()+strlen("misc=");

    // todo: 判断search类型
    MiscAction* action;
    for (int i=0; i<batch; i++) {
        if (!strcasecmp(actionCmd, "copy")) {
            action = new CopyAction();
        } else if (!strcasecmp(actionCmd, "transform")) {
            action = new TransformAction(actionStr);
        } else {
            action = NULL;
        }

        if (action != NULL)
            actionList.push_back(action);
    }

    return actionList;
}

void MiscAction::showHelp(void)
{
    printf("\n");
    printf("misc=<ACTION> [ARGS]\n");
    printf("\n");
    printf("All misc actions are:\n");
    printf("\n");
    printf("  copy               Copy PointCloud\n");
    printf("\n");
    printf("  transform          Transform PointCloud\n");
    printf("    trans=0.0,0.0,0.0        the translation\n");
    printf("    quat=x,y,z,w             rotation as quaternion\n");
    printf("    axis=ax,ay,az,theta      rotation in axis-angle form\n");
    printf("    scale=1,0,1.0,1.0        scale each dimension with these values\n");
    printf("    matrix=v1,v2,...,v8,v9   a 3x3 affine transform\n");
    printf("    matrix=v1,v2,...,v15,v16 a 4x4 transformation matrix\n");
    printf("\n");
}

#endif //__MISC_ACTION_H__