#ifndef __ACTION_GRAPH_H__
#define __ACTION_GRAPH_H__

#include "action/Action.h"
#include "digraph/digraph.hh"
#include "digraph/digraphop.hh"
#include <pcl/console/time.h>
#include <pcl/console/print.h>

typedef digraph<Action*, PCW*>    ActionGraph;

// Deep-First Search
int DFS(ActionGraph actionGraph)
{
    schedule<Action*, PCW*> s = dfschedule(reverse(actionGraph));

    pcl::console::print_info("\n");
    for (const auto& action : s.elements()) {
        std::vector<PCW*> in, out;

        for (const auto& e : actionGraph.rconnections(action)) {
            in.push_back(e.second);
        }

        for (const auto& e : actionGraph.connections(action)) {
            out.push_back(e.second);
        }

        // 输出为空，且非sink action；或者输入为空，且非source action，则报错
        if (out.empty() && action->getClass() != Action::SINK)
            pcl::console::print_info("Output cannot be empty!\n");
        else if (in.empty() && action->getClass() != Action::SOURCE)
            pcl::console::print_error("Input cannot be empty!\n");
        else {
            pcl::console::TicToc tt;
            action->dump();
            tt.tic();
            action->doit(in, out);
            pcl::console::print_info("use time: ");
            tt.toc_print();
            pcl::console::print_info("\n");
        }
    }

    return 0;
}

#endif //__ACTION_GRAPH_H__