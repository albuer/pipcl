#ifndef __SOURCE_ACTION_H__
#define __SOURCE_ACTION_H__

class SourceAction : public Action
{
public:
    SourceAction(std::string n):Action(n, Action::SOURCE) {}
    static std::vector<Action*> parse(std::vector<const char*>& actionStr);
    static void showHelp(void);
};

#include "FileReaderAction.h"
#include "ShapeAction.h"

std::vector<Action*> SourceAction::parse(std::vector<const char*>& actionStr)
{
    std::vector<Action*> actionList;
    const char* source = actionStr.front()+strlen("source=");

    if (!strcasecmp(source, "grabber")) {
        // 从深度相机获取点云数据
    } else if (!strcasecmp(source, "Sphere") ||
               !strcasecmp(source, "Cube") ||
               !strcasecmp(source, "Line") ||
               !strcasecmp(source, "Plane") ||
               !strcasecmp(source, "Circle") ||
               !strcasecmp(source, "Noise")) {
        // 生成指定形状的点云数据
        actionList.push_back(new ShapeAction(actionStr));
    } else {
        // 从本地文件读取点云数据
        char* filelist = new char[strlen(source)+1];
        strcpy(filelist, source);

        char *token = strtok(filelist, ",");
        while (token != NULL) {
            // todo: 需要判断source类型 file/grabber/make
            actionList.push_back(new FileReaderAction(token));

            token = strtok(NULL, ",");
        }
        delete []filelist;
    }

    return actionList;
}

void SourceAction::showHelp(void)
{
    printf("\n");
    printf("source=<TYPE> [ARGS]\n");
    printf("\n");
    printf("All source type are:\n");
    printf("\n");
    printf("  File                      Support pcd/ply/ifs file\n");
    printf("\n");
    printf("  Line                      Create line\n");
    printf("    point1=1,0,0              point 1 (x, y, z)\n");
    printf("    point2=0,1,0              point 2 (x, y, z)\n");
    printf("    density=1                 points per cm\n");
    printf("\n");
    printf("  Plane                     Create plane\n");
    printf("    origin=0,0,0              origin point (x, y, z)\n");
    printf("    point1=1,0,0              point 1 (x, y, z)\n");
    printf("    point2=0,1,0              point 2 (x, y, z)\n");
    printf("    density=1                 points per cm\n");
    printf("\n");
    printf("  Circle                    Create circle\n");
    printf("    center=0,0,0              circle center (x,y,z)\n");
    printf("    radius=1                  circle radius\n");
    printf("    density=1                 points per cm\n");
    printf("\n");
    printf("  Sphere                    Create sphere\n");
    printf("    center=0,0,0              circle center (x,y,z)\n");
    printf("    radius=1                  circle radius\n");
    printf("    density=1                 points per cm\n");
    printf("\n");
    printf("  Cube                      Create cube\n");
    printf("    xrange=-1,1               x range (left, right)\n");
    printf("    yrange=-1,1               y range (left, right)\n");
    printf("    zrange=-1,1               z range (left, right)\n");
    printf("    density=1                 points per cm\n");
    printf("\n");
    printf("  Noise                     Create noise\n");
    printf("    xrange=-1,1               x range (left, right)\n");
    printf("    yrange=-1,1               y range (left, right)\n");
    printf("    zrange=-1,1               z range (left, right)\n");
    printf("    count=100                 point count\n");
    printf("\n");
    printf("  Grabber\n");
    printf("\n");
}

#endif //__SOURCE_ACTION_H__