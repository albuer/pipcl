#ifndef __REGISTRATION_ACTION_H__
#define __REGISTRATION_ACTION_H__

class RegistrationAction : public Action
{
public:
    RegistrationAction(std::string n):Action(n, Action::REGISTRATION) {}
    static std::vector<Action*> parse(std::vector<const char*>& actionStr, int batch);
    static void showHelp(void);
};

#include "IcpAction.h"

std::vector<Action*> RegistrationAction::parse(std::vector<const char*>& actionStr, int batch) {
    std::vector<Action*> actionList;
    const char* filter = actionStr.front()+strlen("registration=");

    RegistrationAction* action;
    if (!strcasecmp(filter, "ICP")) {
        action = new IcpAction(actionStr);
    } else {
        action = NULL;
    }

    if (action != NULL)
        actionList.push_back(action);

    return actionList;
}

void RegistrationAction::showHelp(void)
{
    printf("\n");
    printf("registration=<ACTION> [ARGS]\n");
    printf("\n");
    printf("All registration actions are:\n");
    printf("\n");
    printf("  ICP                Iterative Closest Point\n");
    printf("    iterations=50      the maximum number of iterations the internal optimization should run for\n");
    printf("    dist=0.05          the maximum distance threshold between a point and its nearest neighbor\n");
    printf("                       correspondent in order to be considered in the alignment process\n");
    printf("    rans=0.05          the inlier distance threshold for the internal RANSAC outlier rejection loop\n");
    printf("    epsilon=0.000001   the transformation epsilon in order for an optimization to be considered\n");
    printf("                       as having converged to the final solution\n");
    printf("    NonLinear=0        Linear (0) or Non-Linear (1)\n");
    printf("\n");
}

#endif //__REGISTRATION_ACTION_H__