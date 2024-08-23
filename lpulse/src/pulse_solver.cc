#include <list>

#include "ap_path.h"
#include "bp_path.h"
#include "pulse_solver.h"
#include "shortest_path.h"

namespace {

void PrintSrlgs(const std::vector<int>& srlgs) {
    for (int srlg : srlgs) {
        std::cout << srlg << " ";
    }
    std::cout << "\n";
}

void PrintSrlgs(const std::unordered_set<int>& srlgs) {
    for (int srlg : srlgs) {
        std::cout << srlg << " ";
    }
    std::cout << "\n";
}

}  // namespace

Path Pulse::FindPath(const Flow &flow) {
    clock_t start_time;
    clock_t end_time;
    start_time = clock();
    AStar a_star_delay(graph_, LinkDelay);
    a_star_delay.InitWithDst(flow.to);
    AStar a_star_cost(graph_, LinkCost);
    a_star_cost.InitWithDst(flow.to);
    GenericAp ap;
    ap.Init(graph_, nullptr, a_star_cost.GetCostVector(),
            a_star_delay.GetCostVector());
    double min_cost = ap.FindOptPath(flow, kMaxValue, ap_info_, bp_info_);
    Path result;
    if (min_cost < kMaxValue) {
        result.path_link = ap.GetApPath();
        result.CompletePath();
    }
    end_time = clock();
    // std::cout << "Pulse takes: "
    //           << double(end_time - start_time) / CLOCKS_PER_SEC * 1000
    //           << "(ms).\n";
    return result;
}

Path MulGraphPulse::FindPath(const Flow &flow)
{
    clock_t start_time;
    clock_t end_time;
    start_time = clock();
    graph_->BuildMulGraph(flow);
    const std::vector<Link> &Mul_links = graph_->GetMulLink();
    // ap_info_.total_time += clock() - start_time;
    Graph mulgraph(Mul_links);
    mul_graph_ = &mulgraph;
    // start_time = clock();
    Flow mul_flow(flow.id, flow.from, flow.to + flow.SF_number * (graph_->GetMaxNodeId() + 1),
                flow.delay_lb, flow.delay_ub, flow.bandwidth, 
                flow.is_diff, flow.diff, flow.type, flow.opt_cost);
    AStar a_star_delay(mul_graph_, LinkDelay);
    a_star_delay.InitWithDst(mul_flow.to);
    AStar a_star_cost(mul_graph_, LinkCost);
    a_star_cost.InitWithDst(mul_flow.to);
    MulGraphAp ap;
    ap.Init(mul_graph_, nullptr, flow, a_star_cost.GetCostVector(),
            a_star_delay.GetCostVector());
    ap_info_.total_time += clock() - start_time;
    double min_cost = ap.FindOptPath(mul_flow, kMaxValue, ap_info_, bp_info_);
    Path result;
    if (min_cost < kMaxValue) {
        std::vector<Link *> cur_path = ap.GetApPath();
        std::vector<Link *> original_path;
        std::vector<NodeInfo> used_nodes;
        GetOriginalPath(graph_, cur_path, flow, original_path);
        GetUsedNodes(graph_, cur_path, flow, used_nodes);
        result.path_used_function_nodes = used_nodes;
        result.path_link = original_path;
        result.CompletePathwithNodeWeight();
    }
    end_time = clock();
    // std::cout << "Pulse takes: "
    //           << double(end_time - start_time) / CLOCKS_PER_SEC * 1000
    //           << "(ms).\n";
    return result;
}