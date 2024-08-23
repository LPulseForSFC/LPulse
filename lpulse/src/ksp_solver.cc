#include "bp_path.h"
#include "ksp_solver.h"
#include "shortest_path.h"
#include <algorithm>
#include <vector>

Path DelayKsp::FindPath(const Flow &flow)
{
    clock_t start_time;
    clock_t end_time;
    start_time = clock();
    KShortestPath ksp(graph_, LinkDelay);
    double delay = ksp.Init(flow.from, flow.to);
    int num_iterations = 0;
    double min_cost = kMaxValue;
    Path result;
    while (flow.CheckDelayUb(delay))
    {
        if (flow.CheckDelayLb(delay))
        {
            const std::vector<Link *> &cur_path = ksp.GetPath();
            double cost = ComputeCost(cur_path);
            if (cost < min_cost)
            {
                result.path_link = cur_path;
                min_cost = cost;
            }
        }
        ++ap_info_.iteration_num;
        delay = ksp.FindNextPath();
    }
    // std::cout << "Total number of iterations: " << num_iterations << "\n";
    end_time = clock();
    ap_info_.total_time = end_time - start_time;
    std::cout << "Delay KSP takes: "
              << double(end_time - start_time) / CLOCKS_PER_SEC * 1000
              << "(ms).\n";
    if (min_cost < kMaxValue)
    {
        result.CompletePath();
    }
    return result;
}

PathPair DelayKsp::FindPathPair(const Flow &flow)
{
    clock_t start_time;
    clock_t end_time;
    start_time = clock();
    KShortestPath ksp(graph_, LinkDelay);
    KspBp ksp_bp(graph_);
    clock_t bp_start_time;
    clock_t bp_end_time;
    double bp_time_ms = 0;
    double delay = ksp.Init(flow.from, flow.to);
    // int num_iterations = 0;
    double min_cost = kMaxValue;
    PathPair result;
    while (flow.CheckDelayUb(delay))
    {
        if (flow.CheckDelayLb(delay))
        {
            const std::vector<Link *> &cur_path = ksp.GetPath();
            double cost = ComputeCost(cur_path);
            if (cost < min_cost)
            {
                Flow bp_flow = flow;
                bp_flow.delay_lb = delay - flow.diff > flow.delay_lb ? delay - flow.diff : flow.delay_lb;
                bp_flow.delay_ub = delay + flow.diff < flow.delay_ub ? delay + flow.diff : flow.delay_ub;
                bp_start_time = clock();
                double bp_delay = ksp_bp.FindBpPath(cur_path, bp_flow, bp_info_.iteration_num);
                bp_end_time = clock();
                bp_info_.total_time += bp_end_time - bp_start_time;
                bp_time_ms += static_cast<double>(bp_end_time - bp_start_time) /
                              CLOCKS_PER_SEC * 1000;
                if (bp_delay < kMaxValue)
                {
                    result.ap_path.path_link = cur_path;
                    result.bp_path.path_link = ksp_bp.GetBpPath();
                    min_cost = cost;
                }
            }
        }
        ++ap_info_.iteration_num;
        delay = ksp.FindNextPath();
    }
    // std::cout << "Total number of AP iterations: " << num_iterations << "\n";
    end_time = clock();
    ap_info_.total_time += end_time - start_time - bp_info_.total_time;
    std::cout << "Delay KSP takes: "
              << double(end_time - start_time) / CLOCKS_PER_SEC * 1000
              << "(ms).\n";
    // std::cout << "BP search takes: " << bp_time_ms << "(ms).\n";
    if (min_cost < kMaxValue)
    {
        result.ap_path.CompletePath();
        result.bp_path.CompletePath();
    }
    return result;
}

Path CostKsp::FindPath(const Flow &flow)
{
    clock_t start_time;
    clock_t end_time;
    start_time = clock();
    KShortestPath ksp(graph_, LinkCost);
    double cost = ksp.Init(flow.from, flow.to);
    int num_iterations = 0;
    Path result;
    while (cost < kMaxValue)
    {
        const std::vector<Link *> &cur_path = ksp.GetPath();
        double delay = ComputeDelay(cur_path);
        if (flow.CheckDelayUb(delay) && flow.CheckDelayLb(delay))
        {
            result.path_link = cur_path;
            result.CompletePath();
            break;
        }
        ++ap_info_.iteration_num;
        ++num_iterations;
        cost = ksp.FindNextPath();
    }
    // std::cout << "Total number of iterations: " << num_iterations << "\n";
    end_time = clock();
    ap_info_.total_time = end_time - start_time;
    std::cout << "CostKSP takes: "
              << double(end_time - start_time) / CLOCKS_PER_SEC * 1000
              << "(ms).\n";
    ap_path_ = &result;
    return result;
}

PathPair CostKsp::FindPathPair(const Flow &flow)
{
    clock_t start_time;
    clock_t end_time;
    start_time = clock();
    KShortestPath ksp(graph_, LinkCost);
    KspBp ksp_bp(graph_);
    clock_t bp_start_time;
    clock_t bp_end_time;
    double bp_time_ms = 0;
    double cost = ksp.Init(flow.from, flow.to);
    int num_iterations = 0;
    PathPair result;
    while (cost < kMaxValue)
    {
        ++num_iterations;
        const std::vector<Link *> &cur_path = ksp.GetPath();
        double delay = ComputeDelay(cur_path);
        ++ap_info_.iteration_num;
        if (flow.CheckDelayUb(delay) && flow.CheckDelayLb(delay))
        {
            Flow bp_flow = flow;
            bp_flow.delay_lb = delay - flow.diff > flow.delay_lb ? delay - flow.diff : flow.delay_lb;
            bp_flow.delay_ub = delay + flow.diff < flow.delay_ub ? delay + flow.diff : flow.delay_ub;
            bp_start_time = clock();
            double bp_delay = ksp_bp.FindBpPath(cur_path, bp_flow, bp_info_.iteration_num);
            bp_end_time = clock();
            bp_info_.total_time += bp_end_time - bp_start_time;
            bp_time_ms += static_cast<double>(bp_end_time - bp_start_time) /
                          CLOCKS_PER_SEC * 1000;
            if (bp_delay < kMaxValue)
            {
                result.ap_path.path_link = cur_path;
                result.ap_path.CompletePath();
                result.bp_path.path_link = ksp_bp.GetBpPath();
                result.bp_path.CompletePath();
                break;
            }
        }
        cost = ksp.FindNextPath();
    }
    ap_path_ = &result.ap_path;
    bp_path_ = &result.bp_path;
    end_time = clock();
    // std::cout << "Total number of AP iterations: " << num_iterations << "\n";
    // std::cout << "BP search takes: " << bp_time_ms << "(ms).\n";
    std::cout << "CostKSP (including BP search) takes: "
              << double(end_time - start_time) / CLOCKS_PER_SEC * 1000
              << "(ms).\n";
    ap_info_.total_time = end_time - start_time - bp_info_.total_time;
    return result;
}

Path CostKspPulse::FindPath(const Flow &flow)
{
    clock_t start_time;
    clock_t end_time;
    start_time = clock();
    KShortestPath ksp(graph_, LinkCost);
    double cost = ksp.Init(flow.from, flow.to);
    ksp.AddSecondCostFunction(LinkDelay, flow.delay_ub);
    int num_iterations = 0;
    Path result;
    while (cost < kMaxValue)
    {
        const std::vector<Link *> &cur_path = ksp.GetPath();
        double delay = ComputeDelay(cur_path);
        if (flow.CheckDelayUb(delay) && flow.CheckDelayLb(delay))
        {
            result.path_link = cur_path;
            result.CompletePath();
            break;
        }
        ++num_iterations;
        ++ap_info_.iteration_num;
        cost = ksp.FindNextPath();
    }
    // std::cout << "Total number of iterations: " << num_iterations << "\n";
    end_time = clock();
    ap_info_.total_time = end_time - start_time;
    std::cout << "CostKSPPulse takes: "
              << double(end_time - start_time) / CLOCKS_PER_SEC * 1000
              << "(ms).\n";
    ap_path_ = &result;
    return result;
}

PathPair CostKspPulse::FindPathPair(const Flow &flow)
{
    clock_t start_time;
    clock_t end_time;
    start_time = clock();
    KShortestPath ksp(graph_, LinkCost);
    DisjointBp pulse_bp(graph_);
    AStar a_star(graph_, LinkDelay);
    a_star.InitWithDst(flow.to);
    SrlgDisjointBp conflict_set_bp(graph_, a_star.GetCostVector());
    clock_t bp_start_time;
    clock_t bp_end_time;
    double bp_time_ms = 0;
    double cost = ksp.Init(flow.from, flow.to);
    ksp.AddSecondCostFunction(LinkDelay, flow.delay_ub);
    int num_iterations = 0;
    PathPair result;
    std::vector<ConflictSet> conflict_sets;
    conflict_sets.reserve(20);
    while (cost < kMaxValue)
    {
        const std::vector<Link *> &cur_path = ksp.GetPath();
        double delay = ComputeDelay(cur_path);
        if (flow.CheckDelayUb(delay) && flow.CheckDelayLb(delay))
        {
            bool skip = false;
            for (ConflictSet &cs : conflict_sets)
            {
                ConflictStatus status(&cs);
                status.Init(cur_path);
                if (status.CoverConflictSet())
                {
                    skip = true;
                    break;
                }
            }
            if (!skip)
            {
                Flow bp_flow = flow;
                bp_flow.delay_lb = delay - flow.diff > flow.delay_lb ? delay - flow.diff : flow.delay_lb;
                bp_flow.delay_ub = delay + flow.diff < flow.delay_ub ? delay + flow.diff : flow.delay_ub;
                bp_start_time = clock();
                double bp_delay = pulse_bp.FindBpPath(cur_path, bp_flow, bp_info_.iteration_num);
                bp_end_time = clock();
                bp_info_.total_time += bp_end_time - bp_start_time;
                bp_time_ms += static_cast<double>(bp_end_time - bp_start_time) /
                              CLOCKS_PER_SEC * 1000;
                if (bp_delay < kMaxValue)
                {
                    result.ap_path.path_link = cur_path;
                    result.ap_path.CompletePath();
                    result.bp_path.path_link = pulse_bp.GetBpPath();
                    result.bp_path.CompletePath();
                    break;
                }
                else
                {
                    double bp_delay_original_flow =
                        conflict_set_bp.FindBpPath(cur_path, flow, bp_info_.iteration_num);
                    if (bp_delay_original_flow == kMaxValue)
                    {
                        conflict_sets.push_back(
                            conflict_set_bp.GetConflictSet());
                        ksp.AddConflictSet(conflict_sets.back());
                    }
                }
            }
        }
        ++ap_info_.iteration_num;
        ++num_iterations;
        cost = ksp.FindNextPath();
    }
    // std::cout << "Total number of AP iterations: " << num_iterations << "\n";
    end_time = clock();
    // std::cout << "BP search takes: " << bp_time_ms << "(ms).\n";
    std::cout << "CostKSPPulse (including BP search) takes: "
              << double(end_time - start_time) / CLOCKS_PER_SEC * 1000
              << "(ms).\n";
    ap_info_.total_time = end_time - start_time - bp_info_.total_time;
    return result;
}

Path LagrangianKsp::FindPath(const Flow &flow) {
    clock_t start_time;
    clock_t end_time;
    start_time = clock();
    double u;
    double min_cost_delay_ratio = kMaxValue;
    for (Link &link : graph_->GetMutableLinks()) {
        double ratio = link.cost / link.delay;
        if (ratio < min_cost_delay_ratio) {
            min_cost_delay_ratio = ratio;
        }
    }
    if (default_u_ != kMaxValue) {
        u = default_u_;
    } else {
        u = CalculateMultiplier(graph_, flow);
        if (u == kMaxValue) {
            return Path();
        }
    }
    auto weight_func = [u](Link *link) {
        return link->cost + u * link->delay;
    };
    KShortestPath ksp(graph_, weight_func);
    double weight = ksp.Init(flow.from, flow.to);
    int num_iterations = 0;
    Path result;
    double min_cost = kMaxValue;
    double weight_ub = kMaxValue;
    while (weight < weight_ub) {
        const std::vector<Link *> &cur_path = ksp.GetPath();
        double delay = ComputeDelay(cur_path);
        if (flow.CheckDelayUb(delay) && flow.CheckDelayLb(delay)) {
            double cost = ComputeCost(cur_path);
            if (cost < min_cost) {
                result.path_link = cur_path;
                min_cost = cost;
                weight_ub = u < 0 ? cost + u * flow.delay_lb : cost + u * flow.delay_ub;
            }
        }
        ++ap_info_.iteration_num;
        weight = ksp.FindNextPath();
    }
    result.CompletePath();
    ap_path_ = &result;
    // std::cout << "Total number of iterations: " << num_iterations << "\n";
    end_time = clock();
    ap_info_.total_time = end_time - start_time;
    std::cout << "LagrangianKsp with u=" << u << " takes: "
              << double(end_time - start_time) / CLOCKS_PER_SEC * 1000
              << "(ms).\n";
    return result;
}

class Cmp {
 public:
    bool operator()(const Path &a, const Path &b) {
        return ComputeCost(a.path_link) > ComputeCost(b.path_link);
    }
};


// new lagrangian
PathPair LagrangianKsp::FindPathPair(const Flow &flow) {
    clock_t start_time;
    clock_t end_time;
    start_time = clock();
    clock_t bp_start_time;
    clock_t bp_end_time;
    double bp_time_ms = 0;
    double u;
    double min_cost_delay_ratio = kMaxValue;
    for (Link &link : graph_->GetMutableLinks()) {
        double ratio = link.cost / link.delay;
        if (ratio < min_cost_delay_ratio) {
            min_cost_delay_ratio = ratio;
        }
    }
    if (default_u_ != kMaxValue) {
        u = default_u_;
    }
    else {
        u = CalculateMultiplier(graph_, flow);
        if (u == kMaxValue) {
            return PathPair();
        }
    }
    auto weight_func = [u](Link *link) {
        return link->cost + u * link->delay;
    };
    // std::cout << std::endl;
    // std::cout << u << std::endl;
    KShortestPath ksp(graph_, weight_func);
    KspBp ksp_bp(graph_);
    double weight = ksp.Init(flow.from, flow.to);
    double weight_buffer = weight;
    int num_iterations = 0;
    PathPair result;
    double min_cost = kMaxValue;
    double weight_ub = kMaxValue;
    int cnt = 0;
    std::priority_queue<Path, std::vector<Path>, Cmp> stored_path;
    while (true) {
        const std::vector<Link *> &cur_path = ksp.GetPath();
        double delay = ComputeDelay(cur_path);
        if (delay == kMaxValue)
            break;
        if (weight > weight_ub) {
            // std::cout << stored_path.size() << std::endl;
            Path path = stored_path.top();
            stored_path.pop();
            // std::cout << "hello" << std::endl;
            double delay = ComputeDelay(path.path_link);
            Flow bp_flow = flow;
            bp_flow.delay_lb = delay - flow.diff > flow.delay_lb ? delay - flow.diff : flow.delay_lb;
            bp_flow.delay_ub = delay + flow.diff < flow.delay_ub ? delay + flow.diff : flow.delay_ub;
            bp_start_time = clock();
            double bp_delay = ksp_bp.FindBpPath(path.path_link, bp_flow, bp_info_.iteration_num);
            ++cnt;
            bp_end_time = clock();
            bp_info_.total_time += bp_end_time - bp_start_time;
            if (bp_delay < kMaxValue) {
                result.ap_path.path_link = path.path_link;
                result.bp_path.path_link = ksp_bp.GetBpPath();
                result.ap_path.CompletePath();
                result.bp_path.CompletePath();
                end_time = clock();
                // std::cout << "Parse 1 finished!, Iteration time: " << cnt << std::endl; 
                ap_info_.total_time = end_time - start_time - bp_info_.total_time;
                std::cout << "LagrangianKsp with u=" << u << " takes: "
                    << double(end_time - start_time) / CLOCKS_PER_SEC * 1000
                    << "(ms).\n";
                return result;
            }

            double cost;
            if (stored_path.size() == 0) {
                cost = kMaxValue;
            } else {
                cost = stored_path.top().cost;
            }
            weight_ub = u < 0 ? cost + u * flow.delay_lb : cost + u * flow.delay_ub;
        }
        if (flow.CheckDelayUb(delay) && flow.CheckDelayLb(delay)) {
            Path path;
            path.path_link = cur_path;
            path.cost = ComputeCost(path.path_link);
            path.CompletePath();
            stored_path.push(path);
            double cost = stored_path.top().cost;
            weight_ub = u < 0 ? cost + u * flow.delay_lb : cost + u * flow.delay_ub;
        }
        ++ap_info_.iteration_num;
        weight = ksp.FindNextPath();
    }
    // std::cout << "Parse2: " << std::endl;
    bp_start_time = clock();
    // std::cout << "\nsize: " << stored_path.size() << std::endl;
    while (!stored_path.empty()) {
        Path path = stored_path.top();
        stored_path.pop();
        bp_start_time = clock();
        double bp_delay = ksp_bp.FindBpPath(path.path_link, flow, bp_info_.iteration_num);
        bp_end_time = clock();
        bp_info_.total_time += bp_end_time - bp_start_time;
        if (bp_delay < kMaxValue) {
            result.ap_path.path_link = path.path_link;
            result.bp_path.path_link = ksp_bp.GetBpPath();
            ap_path_ = &result.ap_path;
            bp_path_ = &result.bp_path;
            end_time = clock();
            bp_end_time = clock();
            ap_info_.total_time = end_time - start_time - bp_info_.total_time;
            std::cout << "LagrangianKsp with u=" << u << " takes: "
                    << double(end_time - start_time) / CLOCKS_PER_SEC * 1000
                    << "(ms).\n";
            return result;
        }
    }
    return result;
}

Path SfcKsp::FindPath(const Flow& flow)
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
    int new_dst = flow.to + flow.SF_number * (graph_->GetMaxNodeId() + 1);
    KShortestPath ksp(mul_graph_, LinkCost);
    double cost = ksp.Init(flow.from, new_dst);
    int num_iterations = 0;
    Path result;
    while (cost < kMaxValue)
    {
        ++ap_info_.iteration_num;
        ++num_iterations;
        const std::vector<Link *> &cur_path = ksp.GetPath();
        double delay = ComputeDelay(cur_path);
        if (flow.CheckDelayUb(delay) && flow.CheckDelayLb(delay))
        {
            std::vector<Link *> original_path;
            std::vector<NodeInfo> used_nodes;
            GetOriginalPath(graph_, cur_path, flow, original_path);
            GetUsedNodes(graph_, cur_path, flow, used_nodes);
            result.path_used_function_nodes = used_nodes;
            result.path_link = original_path;
            result.CompletePathwithNodeWeight();
            break;
        }
        cost = ksp.FindNextPath();
    }
    // std::cout << "Total number of iterations: " << num_iterations << "\n";
    end_time = clock();
    ap_info_.total_time = end_time - start_time;
    // std::cout << "Original Cost KSP takes: "
    //           << double(end_time - start_time) / CLOCKS_PER_SEC * 1000
    //           << "(ms).\n";
    ap_path_ = &result;
    return result;

}

Path SfcLagrangianKsp::FindPath(const Flow &flow)
{
    clock_t start_time;
    clock_t end_time;
    start_time = clock();
    graph_->BuildMulGraph(flow);
    const std::vector<Link> &Mul_links = graph_->GetMulLink();
    // ap_info_.total_time += clock() - start_time;
    // std::cout<<ap_info_.total_time<<std::endl;
    Graph mulgraph(Mul_links);
    mul_graph_ = &mulgraph;
    // start_time = clock();
    Flow mul_flow(flow.id, flow.from, flow.to + flow.SF_number * (graph_->GetMaxNodeId() + 1),
                flow.delay_lb, flow.delay_ub, flow.bandwidth, 
                flow.is_diff, flow.diff, flow.type, flow.opt_cost);
    double u;
    double min_cost_delay_ratio = kMaxValue;
    for (Link &link : graph_->GetMutableLinks())
    {
        double ratio = link.cost / link.delay;
        if (ratio < min_cost_delay_ratio)
        {
            min_cost_delay_ratio = ratio;
        }
    }
    if (default_u_ != kMaxValue)
    {
        u = default_u_;
    }
    else
    {
        u = CalculateMultiplier(mul_graph_, mul_flow);
        if (u == kMaxValue)
        {
            return Path();
        }
    }
    auto weight_func = [u](Link *link)
    {
        return link->cost + u * link->delay;
    };
    KShortestPath ksp(mul_graph_, weight_func);
    double weight = ksp.Init(mul_flow.from, mul_flow.to);
    int num_iterations = 0;
    Path result;
    double min_cost = kMaxValue;
    double weight_ub = kMaxValue;
    while (weight < weight_ub)
    {
        const std::vector<Link *> &cur_path = ksp.GetPath();
        double delay = ComputeDelay(cur_path);
        if (flow.CheckDelayUb(delay) && flow.CheckDelayLb(delay))
        {
            double cost = ComputeCost(cur_path);
            if (cost < min_cost)
            {
                std::vector<Link *> original_path;
                std::vector<NodeInfo> used_nodes;
                GetOriginalPath(graph_, cur_path, flow, original_path);
                GetUsedNodes(graph_, cur_path, flow, used_nodes);
                result.path_used_function_nodes = used_nodes;
                result.path_link = original_path;
                min_cost = cost;
                weight_ub = u < 0 ? cost + u * flow.delay_lb : cost + u * flow.delay_ub;
            }
        }
        ++ap_info_.iteration_num;
        weight = ksp.FindNextPath();
    }
    result.CompletePathwithNodeWeight();
    ap_path_ = &result;
    // std::cout << "Total number of iterations: " << num_iterations << "\n";
    end_time = clock();
    ap_info_.total_time = end_time - start_time;
    // std::cout << "LagrangianKsp with u=" << u << " takes: "
    //           << double(end_time - start_time) / CLOCKS_PER_SEC * 1000
    //           << "(ms).\n";
    return result;
}