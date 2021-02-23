#include <iostream>
#include "action/Action.h"
#include "ActionGraph.h"

inline std::ostream& operator<<(std::ostream& file, std::vector<const char*>& strVector)
{
    for (size_t i=0; i<strVector.size(); i++)
        file << strVector[i] << " ";

    return file;
}

static std::vector<Action*> tailActions;
static std::stack<std::vector<Action*>> tailStack;

static void link_leafs(ActionGraph& actionGraph, Action* action)
{
    std::vector<Action *> leafs = leaves(actionGraph);
    for (size_t i = 0; i < leafs.size(); i++)
    {
        actionGraph.add(leafs[i], action, new PCW());
    }
    tailActions.clear();
    tailActions.push_back(action);
}

static int parse_action(ActionGraph& actionGraph, const char* str)
{
    static std::vector<const char*> actionStr;
    if (str[0] == ':') {
        if (actionStr.empty()) {
            // 如果没有具体的action cmd，则执行默认的copy action
            actionStr.push_back("misc=copy");
        }

        int batch = 1;
        bool isSource = !strncasecmp(actionStr[0], "source=", 7);
        bool isRegistration = !strncasecmp(actionStr[0], "registration=", 13);

        if (tailActions.empty() && !isSource) {
            std::cerr << "The first action must be source!" << std::endl;
            return -1;
        }

        if (!tailActions.empty())
            batch = tailActions.size();

        // 解析Action
        std::vector<Action*> actionList = Action::parse(actionStr, batch);
        if (actionList.empty()) {
            std::cerr << "Bad action: " << actionStr << std::endl;
            return -2;
        }

        // 更新graph
        for (size_t i=0; i<actionList.size(); i++) {
            if (isSource) {
                // 所有source action都是root
                actionGraph.add(actionList[i]);
            } else if (isRegistration) {
                // 把所有叶子节点都连接到registration action，并做为tail action
                link_leafs(actionGraph, actionList[i]);
            } else {
                actionGraph.add(tailActions[i], actionList[i], new PCW());
            }
        }

        // 处理管道分割符 : :+ :- :=
        switch(str[1]) {
        case '+': // :+ 主分支
            // 把tail action入栈
            tailStack.push(actionList);
        case '\0': // 普通pipe
            tailActions = actionList;
            break;
        case '-': // :- 次分支
            if (tailStack.empty()) {
                std::cerr << "Found mismatched branches: " << str << std::endl;
                return -4;
            }
            // 出栈，并做为tail action
            tailActions = tailStack.top(); tailStack.pop();
            break;
        case '=': // := 分支汇合
            // 新建mix action，让所有叶子节点都连接到mix action，并把mix action做为tail action
            link_leafs(actionGraph, new MixAction());
            break;
        default:
            std::cerr << "Found invalid separator: " << str << std::endl;
            return -3;
        }

        // action处理完毕，清空掉
        actionStr.clear();
    } else {
        actionStr.push_back(str);
    }

    return 0;
}

void print_help(const char* app, const char* actionClass)
{
    if (actionClass == NULL) {
        printf("\n");
        printf("usage: %s [help ActionClass] ActionClass=Action [ARGS] : ActionClass=Action [ARGS] ...\n", app);
        printf("\n");
        printf("All action class are:\n");
        printf("  source\n");
        printf("  sink\n");
        printf("  filter\n");
        printf("  segmentation\n");
        printf("  search\n");
        printf("  feature\n");
        printf("  registration\n");
        printf("  misc\n");
        printf("\n");
        printf("See '%s help ActionClass' for more information on a specific action.\n", app);
        printf("\n");
    } else {
        Action::showHelp(actionClass);
    }
}

int main (int argc, char** argv)
{
    int ret = 0;
    ActionGraph actionGraph;

    if (argc < 2) {
        print_help(argv[0], NULL);
        exit(0);
    }
    for (int i=1; i<argc; i++) {
        if (!strcmp(argv[i], "help") || !strcmp(argv[i], "--help") || !strcmp(argv[i], "-h")) {
            ++i;
            print_help(argv[0], (i<argc)?argv[i]:NULL);
            exit(0);
        } else {
            ret = parse_action(actionGraph, argv[i]);
            if (ret < 0)
                return ret;
        }
    }
    // 完成最后一个action的解析
    ret = parse_action(actionGraph, ":");
    if(ret < 0)
        return ret;

    ret = DFS(actionGraph);

    return ret;
}