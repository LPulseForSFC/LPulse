#include "ap_path.h"

namespace
{

    void Unsort(Graph *graph)
    {
        graph->unsort();
    }

    void SortLinks(const double *min_delay_to_dst, Graph *graph)
    {
        for (Link &link : graph->GetMutableLinks())
        {
            link.weight = link.delay + min_delay_to_dst[link.ter_id];
        }
        graph->SortLinks();
    }

    void SortLinksbyLayer(const double *min_delay_to_dst, const Flow &flow, Graph *graph)
    {
        int original_node_num = (graph->GetMaxNodeId() + 1) / (flow.SF_number + 1);
        for(Link &link: graph->GetMutableLinks())
        {
            link.weight = link.delay + min_delay_to_dst[link.ter_id];
        }
        graph->SortLinksLayer();
    }

    void Print(const std::vector<Link *> &links)
    {
        std::cout << "links: ";
        for (Link *link : links)
        {
            std::cout << link->link_id << " ";
        }
        std::cout << "\n";
    }

    int MaxConflictSetIdx(
        const std::vector<ConflictSet> &conflict_sets)
    {
        int max_size = -1;
        int idx = -1;
        for (int i = 0; i < conflict_sets.size(); ++i)
        {
            const ConflictSet &conflict_set = conflict_sets.at(i);
            if (conflict_set.NumConflicts() > max_size)
            {
                max_size = conflict_set.NumConflicts();
                idx = i;
            }
        }
        return idx;
    }

    double min(double a, double b)
    {
        if(a<b){
            return a;
        }else{
            return b;
        }
    }
} // namespace

void ApPath::Init(Graph *graph, BpPath *bp_path_solver,
                  const double *min_cost_to_dst,
                  const double *min_delay_to_dst)
{
    graph_ = graph;
    bp_path_solver_ = bp_path_solver;
    min_cost_to_dst_ = min_cost_to_dst;
    min_delay_to_dst_ = min_delay_to_dst;
    if (min_delay_to_dst_)
    {
        SortLinks(min_delay_to_dst_, graph_);
    }
}

double GenericAp::FindOptPath(const Flow &flow, double cost_ub, LogInfo &ap_info, LogInfo &bp_info)
{
    clock_t start_time;
    clock_t end_time;
    start_time = clock();
    // Start DFS search
    std::vector<Snapshot> ap_stack;
    ap_stack.reserve(1000000);
    // Create a fake link with delay = 0
    Link fake_link(-1, -1, flow.from, 0, 0, 0, 0);
    Snapshot snapshot;
    snapshot.link = &fake_link;
    snapshot.stage = 0;
    ap_stack.push_back(snapshot);
    // Track the searched path
    std::vector<Link *> ap_path;
    double ap_path_delay = 0;
    double ap_path_cost = 0;
    ap_path.reserve(100);
    std::vector<bool> ap_visited(graph_->GetMaxNodeId() + 1, false);
    double best_cost_so_far = cost_ub;
    // Start BP search
    int num_iterations = 0;
    int bp_time = 0;
    while (!ap_stack.empty())
    {
        // ++num_iterations;
        ++ap_info.iteration_num;
        Snapshot ap_snap = ap_stack.back();
        ap_stack.pop_back();
        Link *ap_link = ap_snap.link;
        NodeId u = ap_link->ter_id;
        if (ap_snap.stage == 1)
        {
            ap_path.pop_back();
            ap_path_delay -= ap_link->delay;
            ap_path_cost -= ap_link->cost;
            ap_visited[u] = false;
        }
        else
        {
            double new_delay = ap_path_delay + ap_link->delay;
            double new_cost = ap_path_cost + ap_link->cost;
            if (u == flow.to)
            {
                if (flow.CheckDelayUb(new_delay) &&
                    flow.CheckDelayLb(new_delay) &&
                    new_cost < best_cost_so_far)
                {
                    if (bp_path_solver_ == nullptr)
                    {
                        best_cost_so_far = new_cost;
                        // 注意bp_path的第一跳是fake_link
                        ap_path_.clear();
                        ap_path_.reserve(ap_path.size());
                        for (int i = 1; i < ap_path.size(); ++i)
                        {
                            ap_path_.push_back(ap_path[i]);
                        }
                        ap_path_.push_back(ap_link);
                    }
                    else
                    {
                        Flow bp_flow = flow;
                        bp_flow.delay_lb =
                            new_delay - flow.diff > flow.delay_lb ? new_delay - flow.diff : flow.delay_lb;
                        bp_flow.delay_ub =
                            new_delay + flow.diff < flow.delay_ub ? new_delay + flow.diff : flow.delay_ub;
                        ap_path.push_back(ap_link);
                        clock_t start_bp = clock();
                        double bp_delay =
                            bp_path_solver_->FindBpPath(ap_path, bp_flow, bp_info.iteration_num);
                        clock_t end_bp = clock();
                        bp_time += end_bp - start_bp;
                        // std::cout << "---search bp path---\n";
                        // Print(ap_path);
                        if (bp_delay < kMaxValue)
                        {
                            double bp_cost =
                                ComputeCost(bp_path_solver_->GetBpPath());
                            // std::cout << "find a bp path\n";
                            // Print(bp_path_solver_->GetBpPath());
                            if (bp_cost >= new_cost)
                            {
                                best_cost_so_far = new_cost;
                                ap_path_.clear();
                                ap_path_.reserve(ap_path.size() - 1);
                                for (int i = 1; i < ap_path.size(); ++i)
                                {
                                    ap_path_.push_back(ap_path[i]);
                                }
                                bp_path_ = bp_path_solver_->GetBpPath();
                            }
                            else
                            {
                                best_cost_so_far = bp_cost;
                                ap_path_ = bp_path_solver_->GetBpPath();
                                bp_path_.clear();
                                bp_path_.reserve(ap_path.size() - 1);
                                for (int i = 1; i < ap_path.size(); ++i)
                                {
                                    bp_path_.push_back(ap_path[i]);
                                }
                            }
                        }
                        ap_path.pop_back();
                    }
                }
            }
            else
            {
                // prune
                if (flow.CheckDelayUb(min_delay_to_dst_[u] + new_delay) &&
                    min_cost_to_dst_[u] + new_cost < best_cost_so_far)
                {
                    const std::vector<Link *> &egress_links =
                        graph_->GetEgressLinks(u);
                    if (!egress_links.empty())
                    {
                        ap_path.push_back(ap_link);
                        ap_path_delay += ap_link->delay;
                        ap_path_cost += ap_link->cost;
                        ap_visited[u] = true;
                        ap_snap.stage = 1;
                        ap_stack.push_back(ap_snap);
                        for (Link *link : egress_links)
                        {
                            if (link->status == Conflict)
                            {
                                continue;
                            }
                            NodeId node_id = link->ter_id;
                            if (ap_visited[node_id])
                            {
                                continue;
                            }
                            Snapshot snap;
                            snap.stage = 0;
                            snap.link = link;
                            ap_stack.push_back(snap);
                        }
                    }
                }
            }
        }
    }
    // std::cout << "Total number of iterations: " << num_iterations << "\n";
    end_time = clock();
    // std::cout << "GenericAp takes: "
    //           << double(end_time - start_time) / CLOCKS_PER_SEC * 1000
    //           << "(ms).\n";
    bp_info.total_time += bp_time;
    ap_info.total_time += end_time - start_time - bp_time;
    return best_cost_so_far;
}

double GenericAp::FindCloseToOptPath(const Flow &flow, double cost_ub, LogInfo &ap_info, LogInfo &bp_info)
{
    // Start DFS search
    clock_t start_time;
    clock_t end_time;
    int bp_time = 0;
    start_time = clock();
    std::vector<Snapshot> ap_stack;
    ap_stack.reserve(1000000);
    // Create a fake link with delay = 0
    Link fake_link(-1, -1, flow.from, 0, 0, 0, 0);
    Snapshot snapshot;
    snapshot.link = &fake_link;
    snapshot.stage = 0;
    ap_stack.push_back(snapshot);
    // Track the searched path
    std::vector<Link *> ap_path;
    double ap_path_delay = 0;
    double ap_path_cost = 0;
    ap_path.reserve(100);
    // Initialize visited_best_cost_
    visited_best_cost_.clear();
    visited_best_cost_.reserve(graph_->GetMaxNodeId() + 1);
    for (int i = 0; i <= graph_->GetMaxNodeId(); ++i)
    {
        visited_best_cost_.emplace_back(flow.delay_ub + 1, kMaxValue);
    }
    std::vector<bool> ap_visited(graph_->GetMaxNodeId() + 1, false);
    double best_cost_so_far = cost_ub;
    // Start BP search
    while (!ap_stack.empty())
    {
        Snapshot ap_snap = ap_stack.back();
        ap_stack.pop_back();
        Link *ap_link = ap_snap.link;
        NodeId u = ap_link->ter_id;
        ++ap_info.iteration_num;
        if (ap_snap.stage == 1)
        {
            ap_path.pop_back();
            ap_path_delay -= ap_link->delay;
            ap_path_cost -= ap_link->cost;
            ap_visited[u] = false;
        }
        else
        {
            double new_delay = ap_path_delay + ap_link->delay;
            double new_cost = ap_path_cost + ap_link->cost;
            if (u == flow.to)
            {
                if (flow.CheckDelayUb(new_delay) &&
                    flow.CheckDelayLb(new_delay) &&
                    new_cost < best_cost_so_far)
                {
                    if (bp_path_solver_ == nullptr)
                    {
                        best_cost_so_far = new_cost;
                        // 注意bp_path的第一跳是fake_link
                        ap_path_.clear();
                        ap_path_.reserve(ap_path.size());
                        for (int i = 1; i < ap_path.size(); ++i)
                        {
                            ap_path_.push_back(ap_path[i]);
                        }
                        ap_path_.push_back(ap_link);
                    }
                    else
                    {
                        Flow bp_flow = flow;
                        bp_flow.delay_lb =
                            new_delay - flow.diff > flow.delay_lb ? new_delay - flow.diff : flow.delay_lb;
                        bp_flow.delay_ub =
                            new_delay + flow.diff < flow.delay_ub ? new_delay + flow.diff : flow.delay_ub;
                        ap_path.push_back(ap_link);
                        clock_t start_bp = clock();
                        double bp_delay =
                            bp_path_solver_->FindBpPath(ap_path, bp_flow, bp_info.iteration_num);
                        clock_t end_bp = clock();
                        bp_time += end_bp - start_bp;
                        if (bp_delay < kMaxValue)
                        {
                            double bp_cost =
                                ComputeCost(bp_path_solver_->GetBpPath());
                            if (bp_cost >= new_cost)
                            {
                                best_cost_so_far = new_cost;
                                ap_path_.clear();
                                ap_path_.reserve(ap_path.size() - 1);
                                for (int i = 1; i < ap_path.size(); ++i)
                                {
                                    ap_path_.push_back(ap_path[i]);
                                }
                                bp_path_ = bp_path_solver_->GetBpPath();
                            }
                            else
                            {
                                best_cost_so_far = bp_cost;
                                ap_path_ = bp_path_solver_->GetBpPath();
                                bp_path_.clear();
                                bp_path_.reserve(ap_path.size() - 1);
                                for (int i = 1; i < ap_path.size(); ++i)
                                {
                                    bp_path_.push_back(ap_path[i]);
                                }
                            }
                        }
                        ap_path.pop_back();
                    }
                }
            }
            else
            {
                // prune
                if (flow.CheckDelayUb(min_delay_to_dst_[u] + new_delay) &&
                    min_cost_to_dst_[u] + new_cost < best_cost_so_far &&
                    visited_best_cost_[u][new_delay] > new_cost)
                {
                    visited_best_cost_[u][new_delay] = new_cost;
                    const std::vector<Link *> &egress_links =
                        graph_->GetEgressLinks(u);
                    if (!egress_links.empty())
                    {
                        ap_path.push_back(ap_link);
                        ap_path_delay += ap_link->delay;
                        ap_path_cost += ap_link->cost;
                        ap_visited[u] = true;
                        ap_snap.stage = 1;
                        ap_stack.push_back(ap_snap);
                        for (Link *link : egress_links)
                        {
                            if (link->status == Conflict)
                            {
                                continue;
                            }
                            NodeId node_id = link->ter_id;
                            if (ap_visited[node_id])
                            {
                                continue;
                            }
                            Snapshot snap;
                            snap.stage = 0;
                            snap.link = link;
                            ap_stack.push_back(snap);
                        }
                    }
                }
            }
        }
    }
    end_time = clock();
    // std::cout << "GenericAp takes: "
    //           << double(end_time - start_time) /  * 1000
    //           << "(ms).\n";
    bp_info.total_time += bp_time;
    ap_info.total_time += end_time - start_time - bp_time;
    return best_cost_so_far;
}

void MaskAp::GetDelayCostArray(const Flow& flow)
{
    DelayArray.reserve(graph_->GetMaxNodeId()+1);
    CostArray.reserve(graph_->GetMaxNodeId()+1);
    for(int i = 0; i < graph_->GetMaxNodeId()+1; i++)
    {
        DelayArray.push_back(kMaxValue);
        CostArray.push_back(kMaxValue);
    }
    DelayArray[flow.to] = 0;
    CostArray[flow.to] = 0;
    for(std::unordered_set<int>::const_iterator it = flow.Subset_sequence.back().begin();it!=flow.Subset_sequence.back().end();it++)
    {
        DelayArray[*it] = node_to_min_delay_to_dst_.back()[flow.to][*it];
        CostArray[*it] = node_to_min_cost_to_dst_.back()[flow.to][*it];
    }
    for(int i = flow.SF_number-2; i >=0; i--)
    {
        for(std::unordered_set<int>::const_iterator it = flow.Subset_sequence[i].begin();it!=flow.Subset_sequence[i].end();it++)
        {
            for(std::unordered_set<int>::const_iterator it_ = flow.Subset_sequence[i+1].begin();it_!=flow.Subset_sequence[i+1].end();it_++)
            {
                DelayArray[*it] = min(DelayArray[*it],DelayArray[*it_] + node_to_min_delay_to_dst_[i+1][*it_][*it]);
                CostArray[*it] = min(CostArray[*it],CostArray[*it_] + node_to_min_cost_to_dst_[i+1][*it_][*it]);
            }
        }
    }
}
void MaskAp::Init(Graph *graph, BpPath *bp_path_solver,const Flow &flow,
                const std::vector<std::unordered_map<NodeId,const double *>> node_to_min_cost_to_dst,
                const std::vector<std::unordered_map<NodeId,const double *>> node_to_min_delay_to_dst)
{
    graph_ = graph;
    bp_path_solver_ = bp_path_solver;
    node_to_min_cost_to_dst_ = node_to_min_cost_to_dst;
    node_to_min_delay_to_dst_ = node_to_min_delay_to_dst;
    // if (node_to_min_delay_to_dst_.back()[flow.to])
    // {
    //     SortLinksbyMark(node_to_min_delay_to_dst_.back()[flow.to],flow,graph_);
    // }
}
double MaskAp::FindOptPath(const Flow &flow, double cost_ub, LogInfo &ap_info, LogInfo &bp_info)
{
    clock_t start_time;
    clock_t end_time;
    start_time = clock();
    // Start DFS search
    std::vector<FlagSnapshot> ap_stack;
    ap_stack.reserve(1000000);
    // Create a fake link with delay = 0
    Link fake_link(-1, -1, flow.from, 0, 0, 0, 0);
    FlagSnapshot flagsnapshot;
    flagsnapshot.link = &fake_link;
    flagsnapshot.stage = 0;
    flagsnapshot.flag = false;
    flagsnapshot.step = 0;
    ap_stack.push_back(flagsnapshot);
    if (flow.SF_number > 0 && flow.node_function_hash[flow.from][0] == 1){
        flagsnapshot.flag = true;
        flagsnapshot.step = 1;
        ap_stack.push_back(flagsnapshot);
    }
    // Track the searched path
    std::vector<Link *> ap_path;
    double ap_path_delay = 0;
    double ap_path_cost = 0;
    // Track the next function to be visited
    // 0 - flow.SF_number-1 means the service function, flow.SF_number means the dst
    // step = flow.SF_number means all the service function has been satisfied
    int step = 0;
    ap_path.reserve(100);
    double best_cost_so_far = cost_ub;
    GetDelayCostArray(flow);
    // Start BP search
    int num_iterations = 0;
    int bp_time = 0;
    std::vector<NodeId> used_nodes;
    used_nodes.reserve(flow.SF_number);
    visited_best_cost_.clear();
    visited_best_cost_ = std::vector<std::vector<std::vector<int>>> (graph_->GetMaxNodeId() + 1, 
                                                                    std::vector<std::vector<int>> (flow.SF_number+1,
                                                                    std::vector<int> (flow.delay_ub + 1, kMaxValue)));
    while (!ap_stack.empty())
    {
        // for(int i = 0; i<ap_path.size();i++)
        // {
        //     std::cout<<ap_path[i]->link_id<<" ";
        // }
        ++ap_info.iteration_num;
        FlagSnapshot ap_snap = ap_stack.back();
        ap_stack.pop_back();
        Link *ap_link = ap_snap.link;
        // std::cout<<ap_link->link_id<<std::endl;
        // std::cout<<"flag"<<ap_snap.flag<<std::endl;
        NodeId u = ap_link->ter_id;
        if (ap_snap.stage == 1)
        {
            ap_path.pop_back();
            ap_path_delay -= ap_link->delay;
            ap_path_cost -= ap_link->cost;
            if(ap_snap.flag == true){
                step -= 1;
                ap_path_delay -= graph_->GetNodeInfo(u).delay;
                ap_path_cost -= graph_->GetNodeInfo(u).cost;
                used_nodes.pop_back();
            }
        }
        else
        {
            double new_delay = ap_path_delay + ap_link->delay;
            double new_cost = ap_path_cost + ap_link->cost;
            int new_step = step;
            if(ap_snap.flag == true)
            {
                new_step = step + 1;
                new_delay += graph_->GetNodeInfo(u).delay;
                new_cost += graph_->GetNodeInfo(u).cost;
            }

                // std::cout<<"Yes"<<std::endl;
                // std::cout<<"Size"<<ap_path.size()<<std::endl;
                // std::cout<<"ap_link"<<ap_link->link_id<<std::endl;
                // std::cout<<"ap_link source"<<ap_link->source_id<<std::endl;
                // std::cout<<"u"<<u<<std::endl;
                // std::cout<<std::endl;
                if (u == flow.to &&
                flow.CheckDelayUb(new_delay) &&
                    flow.CheckDelayLb(new_delay) &&
                    new_cost < best_cost_so_far  &&
                    new_step == flow.SF_number)
                {
                    if (bp_path_solver_ == nullptr)
                    {
                        best_cost_so_far = new_cost;
                        // 注意bp_path的第一跳是fake_link
                        ap_path_.clear();
                        ap_path_.reserve(ap_path.size());
                        for (int i = 1; i < ap_path.size(); ++i)
                        {
                            if (ap_path[i]->link_id != -1)
                            {
                                ap_path_.push_back(ap_path[i]);
                            }
                        }
                        ap_path_.push_back(ap_link);
                        used_nodes_.clear();
                        used_nodes_.reserve(flow.SF_number);
                        for(int i = 0; i < used_nodes.size(); i++)
                        {
                            used_nodes_.push_back(NodeInfo(used_nodes[i],graph_->GetNodeInfo(used_nodes[i]).cost,
                                                    graph_->GetNodeInfo(used_nodes[i]).delay, graph_->GetNodeInfo(used_nodes[i]).Capacity));
                        }
                        if(new_step == step + 1)
                        {
                            used_nodes_.push_back(NodeInfo(u,graph_->GetNodeInfo(u).cost,
                                                    graph_->GetNodeInfo(u).delay, graph_->GetNodeInfo(u).Capacity));
                        }
                        
                    }
                    else
                    {
                        Flow bp_flow = flow;
                        bp_flow.delay_lb =
                            new_delay - flow.diff > flow.delay_lb ? new_delay - flow.diff : flow.delay_lb;
                        bp_flow.delay_ub =
                            new_delay + flow.diff < flow.delay_ub ? new_delay + flow.diff : flow.delay_ub;
                        ap_path.push_back(ap_link);
                        clock_t start_bp = clock();
                        double bp_delay =
                            bp_path_solver_->FindBpPath(ap_path, bp_flow, bp_info.iteration_num);
                        clock_t end_bp = clock();
                        bp_time += end_bp - start_bp;
                        // std::cout << "---search bp path---\n";
                        // Print(ap_path);
                        if (bp_delay < kMaxValue)
                        {
                            double bp_cost =
                                ComputeCost(bp_path_solver_->GetBpPath());
                            if (bp_cost >= new_cost)
                            {
                                best_cost_so_far = new_cost;
                                ap_path_.clear();
                                ap_path_.reserve(ap_path.size() - 1);
                                for (int i = 1; i < ap_path.size(); ++i)
                                {
                                    ap_path_.push_back(ap_path[i]);
                                }
                                bp_path_ = bp_path_solver_->GetBpPath();
                            }
                            else
                            {
                                best_cost_so_far = bp_cost;
                                ap_path_ = bp_path_solver_->GetBpPath();
                                bp_path_.clear();
                                bp_path_.reserve(ap_path.size() - 1);
                                for (int i = 1; i < ap_path.size(); ++i)
                                {
                                    bp_path_.push_back(ap_path[i]);
                                }
                            }
                        }
                        ap_path.pop_back();
                    }
                }
            else
            {
                // prune
                double step_delay = kMaxValue;
                double step_cost = kMaxValue;
                for(std::unordered_map<NodeId,const double*>::const_iterator it= node_to_min_cost_to_dst_[new_step].begin();it!=node_to_min_cost_to_dst_[new_step].end();it++)
                {
                    step_delay = min(step_delay,DelayArray[it->first]+node_to_min_delay_to_dst_[new_step][it->first][u]);
                    step_cost = min(step_cost,CostArray[it->first] + node_to_min_cost_to_dst_[new_step][it->first][u]);
                }
                // std::cout<<"new_delay"<<new_delay<<std::endl;
                // std::cout<<"step_delay"<<step_delay<<std::endl;
                // std::cout<<"new_delay + step_delay"<<new_delay + step_delay<<std::endl;
                // std::cout<<"new_step"<<new_step<<std::endl;
                if (flow.CheckDelayUb(step_delay + new_delay) &&
                    step_cost + new_cost < best_cost_so_far &&
                    visited_best_cost_[u][new_step][new_delay] > new_cost)
                {
                    visited_best_cost_[u][new_step][new_delay] = new_cost;
                    const std::vector<Link *> &egress_links =
                        graph_->GetEgressLinks(u);
                    if (!egress_links.empty())
                    {
                        ap_path.push_back(ap_link);
                        if (ap_snap.flag == true){
                            step += 1;
                            ap_path_delay += graph_->GetNodeInfo(u).delay;
                            ap_path_cost += graph_->GetNodeInfo(u).cost;
                            used_nodes.push_back(u);
                        }
                        ap_path_delay += ap_link->delay;
                        ap_path_cost += ap_link->cost;
                        ap_snap.stage = 1;
                        // if(step <  flow.SF_number && flow.node_function_hash[u][step] == 1)
                        // {
                        //     ap_snap.flag = true;
                        //     ap_path_delay += graph_->GetNodeInfo(u).delay;
                        //     ap_path_cost += graph_->GetNodeInfo(u).cost;
                        //     used_nodes.push_back(u);
                        //     step += 1;
                        // }
                        // else{
                        //     ap_snap.flag = false;
                        // }
                        ap_stack.push_back(ap_snap);
                        if(ap_snap.flag == true && step < flow.SF_number && flow.node_function_hash[u][step] == 1)
                        {
                            FlagSnapshot snap;
                            snap.stage = 0;
                            snap.link = new Link(-1, u, u, 0, 0, 0, 0);
                            snap.flag = true;
                            ap_stack.push_back(snap);
                        }
                        for (Link *link : egress_links)
                        {
                            if (link->status == Conflict)
                            {
                                continue;
                            }
                            NodeId node_id = link->ter_id;
                            FlagSnapshot snap;
                            snap.stage = 0;
                            snap.link = link;
                            snap.flag = false;
                            ap_stack.push_back(snap);
                            // std::cout<<"ter node_id:"<<node_id<<std::endl;
                            if(step < flow.SF_number && flow.node_function_hash[node_id][step] == 1)
                            {
                                // std::cout<<"node_id:"<<node_id<<std::endl;
                                snap.flag = true;
                                ap_stack.push_back(snap);
                            }
                        }
                    }
                }
            }
        }
    }
    // std::cout << "Total number of iterations: " << num_iterations << "\n";
    end_time = clock();
    // std::cout << "GenericAp takes: "
    //           << double(end_time - start_time) / CLOCKS_PER_SEC * 1000
    //           << "(ms).\n";
    bp_info.total_time += bp_time;
    ap_info.total_time += end_time - start_time - bp_time;
    return best_cost_so_far;
}

void MulGraphAp::InterEdgeOnly(Graph *graph, const Flow &flow)
{
       std::vector<std::vector<Link*>> inter_layer_edges;
    inter_layer_edges.reserve(flow.SF_number);
    for(int i = 0; i < flow.SF_number; i++)
    {
        inter_layer_edges.push_back({});
    }
    for (Link &link : graph->GetMutableLinks())
    {
        if(link.weight_mask == 2)
        {
            inter_layer_edges[link.layer].push_back(&link);
        }   
    }
    for(int i = 0; i < flow.SF_number; i++)
    {
        std::vector<Link*> links = inter_layer_edges[i];
        int min_cost_link_id = -1;
        int min_delay_link_id = -1;
        double min_cost = kMaxValue;
        double min_delay = kMaxValue;
        for(int j = 0; j < links.size(); j++)
        {
            if(links[j]->cost < min_cost)
            {
                min_cost = links[j]->cost;
                min_cost_link_id = j;
            }
            if(links[j]->delay < min_delay)
            {
                min_delay = links[j]->delay;
                min_delay_link_id = j;
            }
        }
        if (min_cost_link_id == min_delay_link_id)
        {
            links[min_cost_link_id]->weight_mask = 3;
        }
    }
}

void MulGraphAp::Init(Graph *graph, BpPath *bp_path_solver, const Flow &flow,
                  const double *min_cost_to_dst,
                  const double *min_delay_to_dst)
{
    graph_ = graph;
    bp_path_solver_ = bp_path_solver;
    min_cost_to_dst_ = min_cost_to_dst;
    min_delay_to_dst_ = min_delay_to_dst;
    InterEdgeOnly(graph_,flow);
    if (min_delay_to_dst_)
    {
        SortLinksbyLayer(min_delay_to_dst_, flow, graph_);
    }
    // Unsort(graph_);
}

double MulGraphAp::FindOptPath(const Flow &flow, double cost_ub, LogInfo &ap_info, LogInfo &bp_info)
{
    // Start DFS search
    clock_t start_time;
    clock_t end_time;
    int bp_time = 0;
    start_time = clock();
    std::vector<Snapshot> ap_stack;
    ap_stack.reserve(1000000);
    // Create a fake link with delay = 0
    Link fake_link(-1, -1, flow.from, 0, 0, 0, 0);
    Snapshot snapshot;
    snapshot.link = &fake_link;
    snapshot.stage = 0;
    ap_stack.push_back(snapshot);
    // Track the searched path
    std::vector<Link *> ap_path;
    double ap_path_delay = 0;
    double ap_path_cost = 0;
    ap_path.reserve(100);
    // Initialize visited_best_cost_
    visited_best_cost_.clear();
    visited_best_cost_.reserve(graph_->GetMaxNodeId() + 1);
    for (int i = 0; i <= graph_->GetMaxNodeId(); ++i)
    {
        visited_best_cost_.emplace_back(flow.delay_ub + 1, kMaxValue);
    }
    std::vector<bool> ap_visited(graph_->GetMaxNodeId() + 1, false);
    double best_cost_so_far = cost_ub;
    int max_depth = 0;
    // Start BP search
    while (!ap_stack.empty())
    {
        if (ap_stack.size() > max_depth)
        {
            max_depth = ap_stack.size();
        }
        Snapshot ap_snap = ap_stack.back();
        ap_stack.pop_back();
        Link *ap_link = ap_snap.link;
        NodeId u = ap_link->ter_id;
        const std::vector<Link *> &egress_links =
        graph_->GetEgressLinks(u);
        ++ap_info.iteration_num;
        if (ap_snap.stage == 1)
        {
            ap_path.pop_back();
            ap_path_delay -= ap_link->delay;
            ap_path_cost -= ap_link->cost;
            ap_visited[u] = false;
        }
        else
        {
            double new_delay = ap_path_delay + ap_link->delay;
            double new_cost = ap_path_cost + ap_link->cost;
            if (u == flow.to)
            {
                if (flow.CheckDelayUb(new_delay) &&
                    flow.CheckDelayLb(new_delay) &&
                    new_cost < best_cost_so_far)
                {
                    if (bp_path_solver_ == nullptr)
                    {
                        best_cost_so_far = new_cost;
                        // 注意bp_path的第一跳是fake_link
                        ap_path_.clear();
                        ap_path_.reserve(ap_path.size());
                        for (int i = 1; i < ap_path.size(); ++i)
                        {
                            ap_path_.push_back(ap_path[i]);
                        }
                        ap_path_.push_back(ap_link);
                    }
                    else
                    {
                        Flow bp_flow = flow;
                        bp_flow.delay_lb =
                            new_delay - flow.diff > flow.delay_lb ? new_delay - flow.diff : flow.delay_lb;
                        bp_flow.delay_ub =
                            new_delay + flow.diff < flow.delay_ub ? new_delay + flow.diff : flow.delay_ub;
                        ap_path.push_back(ap_link);
                        clock_t start_bp = clock();
                        double bp_delay =
                            bp_path_solver_->FindBpPath(ap_path, bp_flow, bp_info.iteration_num);
                        clock_t end_bp = clock();
                        bp_time += end_bp - start_bp;
                        if (bp_delay < kMaxValue)
                        {
                            double bp_cost =
                                ComputeCost(bp_path_solver_->GetBpPath());
                            if (bp_cost >= new_cost)
                            {
                                best_cost_so_far = new_cost;
                                ap_path_.clear();
                                ap_path_.reserve(ap_path.size() - 1);
                                for (int i = 1; i < ap_path.size(); ++i)
                                {
                                    ap_path_.push_back(ap_path[i]);
                                }
                                bp_path_ = bp_path_solver_->GetBpPath();
                            }
                            else
                            {
                                best_cost_so_far = bp_cost;
                                ap_path_ = bp_path_solver_->GetBpPath();
                                bp_path_.clear();
                                bp_path_.reserve(ap_path.size() - 1);
                                for (int i = 1; i < ap_path.size(); ++i)
                                {
                                    bp_path_.push_back(ap_path[i]);
                                }
                            }
                        }
                        ap_path.pop_back();
                    }
                }
            }
            else
            {
                // prune
                if (flow.CheckDelayUb(min_delay_to_dst_[u] + new_delay) &&
                    min_cost_to_dst_[u] + new_cost < best_cost_so_far &&
                    visited_best_cost_[u][new_delay] > new_cost)
                {
                    visited_best_cost_[u][new_delay] = new_cost;
                    const std::vector<Link *> &egress_links =
                        graph_->GetEgressLinks(u);
                    if (!egress_links.empty())
                    {
                        ap_path.push_back(ap_link);
                        ap_path_delay += ap_link->delay;
                        ap_path_cost += ap_link->cost;
                        ap_visited[u] = true;
                        ap_snap.stage = 1;
                        ap_stack.push_back(ap_snap);
                        if (egress_links.back()->weight_mask >= 3)
                        {
                            Snapshot snap;
                            snap.stage = 0;
                            snap.link = egress_links.back();
                            ap_stack.push_back(snap);
                        }
                        else
                        {
                            for (Link *link : egress_links)
                            {
                                if (link->status == Conflict)
                                {
                                    continue;
                                }
                                NodeId node_id = link->ter_id;
                                if (ap_visited[node_id])
                                {
                                    continue;
                                }
                                Snapshot snap;
                                snap.stage = 0;
                                snap.link = link;
                                ap_stack.push_back(snap);
                            }
                        }
                    }
                }
            }
        }
    }
    end_time = clock();
    // std::cout << "GenericAp takes: "
    //           << double(end_time - start_time) /  * 1000
    //           << "(ms).\n";
    bp_info.total_time += bp_time;
    ap_info.total_time += end_time - start_time - bp_time;
    ap_info.max_stack_depth = max_depth;
    return best_cost_so_far;
}

void MulGraphMaskAp::Init(Graph *graph, BpPath *bp_path_solver, const Flow& flow,
                  const double *min_cost_to_dst,
                  const double *min_delay_to_dst)
{
    graph_ = graph;
    bp_path_solver_ = bp_path_solver;
    min_cost_to_dst_ = min_cost_to_dst;
    min_delay_to_dst_ = min_delay_to_dst;
    step_cost_.clear();
    step_delay_.clear();
    step_cost_.reserve(flow.SF_number + 1);
    step_delay_.reserve(flow.SF_number + 1);
    for (int i= 0; i < flow.SF_number + 1; i++)
    {
        step_cost_.push_back(std::vector<double>(graph->GetMaxNodeId() + 1, kMaxValue));
        step_delay_.push_back(std::vector<double>(graph->GetMaxNodeId() + 1, kMaxValue));
    }
    for(int i = 0; i < (graph->GetMaxNodeId() + 1) * (flow.SF_number + 1); i++)
    {
        int step = i / (graph->GetMaxNodeId() + 1);
        int original = i % (graph->GetMaxNodeId() + 1);
        // std::cout<<"step: "<<step<<"origianl:"<<original<<std::endl;
        step_cost_[step][original] = min_cost_to_dst[i];
        step_delay_[step][original] = min_delay_to_dst[i];
        // std::cout<<min_delay_to_dst[i]<<std::endl;
    }
}

double MulGraphMaskAp::FindOptPath(const Flow &flow, double cost_ub, LogInfo &ap_info, LogInfo &bp_info)
{
    clock_t start_time;
    clock_t end_time;
    start_time = clock();
    // Start DFS search
    std::vector<FlagSnapshot> ap_stack;
    ap_stack.reserve(1000000);
    // Create a fake link with delay = 0
    Link fake_link(-1, -1, flow.from, 0, 0, 0, 0);
    FlagSnapshot flagsnapshot;
    flagsnapshot.link = &fake_link;
    flagsnapshot.stage = 0;
    flagsnapshot.flag = false;
    flagsnapshot.step = 0;
    ap_stack.push_back(flagsnapshot);
    if (flow.SF_number > 0 && flow.node_function_hash[flow.from][0] == 1){
        flagsnapshot.flag = true;
        flagsnapshot.step = 1;
        ap_stack.push_back(flagsnapshot);
    }
    // Track the searched path
    std::vector<Link *> ap_path;
    double ap_path_delay = 0;
    double ap_path_cost = 0;
    // Track the next function to be visited
    // 0 - flow.SF_number-1 means the service function, flow.SF_number means the dst
    // step = flow.SF_number means all the service function has been satisfied
    int step = 0;
    ap_path.reserve(100);
    double best_cost_so_far = cost_ub;
    // Start BP search
    int num_iterations = 0;
    int bp_time = 0;
    std::vector<NodeId> used_nodes;
    used_nodes.reserve(flow.SF_number);
    visited_best_cost_.clear();
    visited_best_cost_ = std::vector<std::vector<std::vector<int>>> (graph_->GetMaxNodeId() + 1, 
                                                                    std::vector<std::vector<int>> (flow.SF_number+1,
                                                                    std::vector<int> (flow.delay_ub + 1, kMaxValue)));
    while (!ap_stack.empty())
    {
        // for(int i = 0; i<ap_path.size();i++)
        // {
        //     std::cout<<ap_path[i]->link_id<<" ";
        // }
        ++ap_info.iteration_num;
        FlagSnapshot ap_snap = ap_stack.back();
        ap_stack.pop_back();
        Link *ap_link = ap_snap.link;
        // std::cout<<ap_link->link_id<<std::endl;
        // std::cout<<"flag"<<ap_snap.flag<<std::endl;
        NodeId u = ap_link->ter_id;
        if (ap_snap.stage == 1)
        {
            ap_path.pop_back();
            ap_path_delay -= ap_link->delay;
            ap_path_cost -= ap_link->cost;
            if(ap_snap.flag == true){
                step -= 1;
                ap_path_delay -= graph_->GetNodeInfo(u).delay;
                ap_path_cost -= graph_->GetNodeInfo(u).cost;
                used_nodes.pop_back();
            }
        }
        else
        {
            double new_delay = ap_path_delay + ap_link->delay;
            double new_cost = ap_path_cost + ap_link->cost;
            int new_step = step;
            if(ap_snap.flag == true)
            {
                new_step = step + 1;
                new_delay += graph_->GetNodeInfo(u).delay;
                new_cost += graph_->GetNodeInfo(u).cost;
            }

                // std::cout<<"Yes"<<std::endl;
                // std::cout<<"Size"<<ap_path.size()<<std::endl;
                // std::cout<<"ap_link"<<ap_link->link_id<<std::endl;
                // std::cout<<"ap_link source"<<ap_link->source_id<<std::endl;
                // std::cout<<"u"<<u<<std::endl;
                // std::cout<<std::endl;
                if (u == flow.to &&
                flow.CheckDelayUb(new_delay) &&
                    flow.CheckDelayLb(new_delay) &&
                    new_cost < best_cost_so_far  &&
                    new_step == flow.SF_number)
                {
                    if (bp_path_solver_ == nullptr)
                    {
                        best_cost_so_far = new_cost;
                        // 注意bp_path的第一跳是fake_link
                        ap_path_.clear();
                        ap_path_.reserve(ap_path.size());
                        for (int i = 1; i < ap_path.size(); ++i)
                        {
                            if (ap_path[i]->link_id != -1)
                            {
                                ap_path_.push_back(ap_path[i]);
                            }
                        }
                        ap_path_.push_back(ap_link);
                        used_nodes_.clear();
                        used_nodes_.reserve(flow.SF_number);
                        for(int i = 0; i < used_nodes.size(); i++)
                        {
                            used_nodes_.push_back(NodeInfo(used_nodes[i],graph_->GetNodeInfo(used_nodes[i]).cost,
                                                    graph_->GetNodeInfo(used_nodes[i]).delay, graph_->GetNodeInfo(used_nodes[i]).Capacity));
                        }
                        if(new_step == step + 1)
                        {
                            used_nodes_.push_back(NodeInfo(u,graph_->GetNodeInfo(u).cost,
                                                    graph_->GetNodeInfo(u).delay, graph_->GetNodeInfo(u).Capacity));
                        }
                        
                    }
                    else
                    {
                        Flow bp_flow = flow;
                        bp_flow.delay_lb =
                            new_delay - flow.diff > flow.delay_lb ? new_delay - flow.diff : flow.delay_lb;
                        bp_flow.delay_ub =
                            new_delay + flow.diff < flow.delay_ub ? new_delay + flow.diff : flow.delay_ub;
                        ap_path.push_back(ap_link);
                        clock_t start_bp = clock();
                        double bp_delay =
                            bp_path_solver_->FindBpPath(ap_path, bp_flow, bp_info.iteration_num);
                        clock_t end_bp = clock();
                        bp_time += end_bp - start_bp;
                        // std::cout << "---search bp path---\n";
                        // Print(ap_path);
                        if (bp_delay < kMaxValue)
                        {
                            double bp_cost =
                                ComputeCost(bp_path_solver_->GetBpPath());
                            if (bp_cost >= new_cost)
                            {
                                best_cost_so_far = new_cost;
                                ap_path_.clear();
                                ap_path_.reserve(ap_path.size() - 1);
                                for (int i = 1; i < ap_path.size(); ++i)
                                {
                                    ap_path_.push_back(ap_path[i]);
                                }
                                bp_path_ = bp_path_solver_->GetBpPath();
                            }
                            else
                            {
                                best_cost_so_far = bp_cost;
                                ap_path_ = bp_path_solver_->GetBpPath();
                                bp_path_.clear();
                                bp_path_.reserve(ap_path.size() - 1);
                                for (int i = 1; i < ap_path.size(); ++i)
                                {
                                    bp_path_.push_back(ap_path[i]);
                                }
                            }
                        }
                        ap_path.pop_back();
                    }
                }
            else
            {
                // prune
                double step_delay = step_delay_[new_step][u];
                double step_cost = step_cost_[new_step][u];
                // std::cout<<"new_delay"<<new_delay<<std::endl;
                // std::cout<<"step_delay"<<step_delay<<std::endl;
                // std::cout<<"new_delay + step_delay"<<new_delay + step_delay<<std::endl;
                // std::cout<<"new_step"<<new_step<<std::endl;
                if (flow.CheckDelayUb(step_delay + new_delay) &&
                    step_cost + new_cost < best_cost_so_far &&
                    visited_best_cost_[u][new_step][new_delay] > new_cost)
                {
                    visited_best_cost_[u][new_step][new_delay] = new_cost;
                    const std::vector<Link *> &egress_links =
                        graph_->GetEgressLinks(u);
                    if (!egress_links.empty())
                    {
                        ap_path.push_back(ap_link);
                        if (ap_snap.flag == true){
                            step += 1;
                            ap_path_delay += graph_->GetNodeInfo(u).delay;
                            ap_path_cost += graph_->GetNodeInfo(u).cost;
                            used_nodes.push_back(u);
                        }
                        ap_path_delay += ap_link->delay;
                        ap_path_cost += ap_link->cost;
                        ap_snap.stage = 1;
                        // if(step <  flow.SF_number && flow.node_function_hash[u][step] == 1)
                        // {
                        //     ap_snap.flag = true;
                        //     ap_path_delay += graph_->GetNodeInfo(u).delay;
                        //     ap_path_cost += graph_->GetNodeInfo(u).cost;
                        //     used_nodes.push_back(u);
                        //     step += 1;
                        // }
                        // else{
                        //     ap_snap.flag = false;
                        // }
                        ap_stack.push_back(ap_snap);
                        if(ap_snap.flag == true && step < flow.SF_number && flow.node_function_hash[u][step] == 1)
                        {
                            FlagSnapshot snap;
                            snap.stage = 0;
                            snap.link = new Link(-1, u, u, 0, 0, 0, 0);
                            snap.flag = true;
                            ap_stack.push_back(snap);
                        }
                        for (Link *link : egress_links)
                        {
                            if (link->status == Conflict)
                            {
                                continue;
                            }
                            NodeId node_id = link->ter_id;
                            FlagSnapshot snap;
                            snap.stage = 0;
                            snap.link = link;
                            snap.flag = false;
                            ap_stack.push_back(snap);
                            // std::cout<<"ter node_id:"<<node_id<<std::endl;
                            if(step < flow.SF_number && flow.node_function_hash[node_id][step] == 1)
                            {
                                // std::cout<<"node_id:"<<node_id<<std::endl;
                                snap.flag = true;
                                ap_stack.push_back(snap);
                            }
                        }
                    }
                }
            }
        }
    }
    // std::cout << "Total number of iterations: " << num_iterations << "\n";
    end_time = clock();
    // std::cout << "GenericAp takes: "
    //           << double(end_time - start_time) / CLOCKS_PER_SEC * 1000
    //           << "(ms).\n";
    bp_info.total_time += bp_time;
    ap_info.total_time += end_time - start_time - bp_time;
    return best_cost_so_far;
}