#ifndef __SINK_ACTION_H__
#define __SINK_ACTION_H__

class SinkAction : public Action
{
public:
    SinkAction(std::string n):Action(n, Action::SINK) {}
    static std::vector<Action*> parse(std::vector<const char*>& actionStr, int batch);
    static void showHelp(void);
};

#include "FileWriterAction.h"
#include "VisualizationAction.h"

std::vector<Action*> SinkAction::parse(std::vector<const char*>& actionStr, int batch) {
    std::vector<Action*> actionList;
    const char* sink = actionStr.front()+strlen("sink=");

    for (int i=0; i<batch; i++) {
        SinkAction* action;
        if (!strcasecmp(sink, "VIS")) {
            action = new VisualizationAction();
        } else {
            action = new FileWriterAction(actionStr);
        }
        
        if (action != NULL)
            actionList.push_back(action);
    }

    return actionList;
}

void SinkAction::showHelp(void)
{
    printf("\n");
    printf("sink=<ACTION> [ARGS]\n");
    printf("\n");
    printf("All sink actions are:\n");
    printf("\n");
    printf("  VIS                visualization\n");
    printf("\n");
    printf("  File               file for write\n");
    printf("    ascii=0            1 for ascii mode, 0 for binary\n");
    printf("\n");
}

#endif //__SINK_ACTION_H__