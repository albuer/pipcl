#ifndef __SEARCH_ACTION_H__
#define __SEARCH_ACTION_H__

class SearchAction : public Action
{
public:
    SearchAction(std::string n):Action(n, Action::SEARCH) {}
    static std::vector<Action*> parse(std::vector<const char*>& actionStr, int batch);
    static void showHelp(void);
};

#include "KdTreeAction.h"

std::vector<Action*> SearchAction::parse(std::vector<const char*>& actionStr, int batch) {
    std::vector<Action*> actionList;
    const char* actionCmd = actionStr.front()+strlen("search=");

    // todo: 判断search类型
    SearchAction* action;
    for (int i=0; i<batch; i++) {
        if (!strcasecmp(actionCmd, "KdTree")) {
            action = new KdTreeAction(actionStr);
        } else {
            action = NULL;
        }

        if (action != NULL)
            actionList.push_back(action);
    }

    return actionList;
}

void SearchAction::showHelp(void)
{
    printf("\n");
    printf("search=<ACTION> [ARGS]\n");
    printf("\n");
    printf("All search actions are:\n");
    printf("\n");
    printf("  KdTree             Kd-Tree Search\n");
    printf("    point=0.0,0.0,0.0  the given query point\n");
    printf("    K=0                the number of neighbors to search for\n");
    printf("    radius=0.0         the radius of the sphere bounding all of p_q's neighbors\n");
    printf("\n");
}

#endif //__SEARCH_ACTION_H__