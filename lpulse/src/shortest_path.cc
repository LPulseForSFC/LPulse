#include "shortest_path.h"

#include <queue>

namespace {

struct NodeCostPair {
    NodeCostPair(NodeId id_in, double cost_in)
        : id(id_in), cost(cost_in) {}
    friend bool operator<(const NodeCostPair &a, const NodeCostPair &b) {
        return a.cost > b.cost;
    }
    NodeId id;
    double cost;
};

struct NodeDelayCostTriple {
    NodeDelayCostTriple(NodeId id_in, double delay_in, double cost_in)
        : id(id_in), delay(delay_in), cost(cost_in) {}
    friend bool operator<(const NodeDelayCostTriple &a,
                          const NodeDelayCostTriple &b) {
        return a.cost > b.cost;
    }
    NodeId id;
    double delay;
    double cost;
};

void ComputeCostFromSrc(Graph* graph, NodeId src, CostFunc f,
                        std::vector<double>* min_cost_from_src) {
    (*min_cost_from_src)[src] = 0;
    std::priority_queue<NodeCostPair, std::vector<NodeCostPair>> heap;
    std::vector<bool> visited(graph->GetMaxNodeId() + 1, false);
    heap.push(NodeCostPair(src, 0));
    while (!heap.empty()) {
        NodeId u = heap.top().id;
        double weight_u_to_dst = heap.top().cost;
        heap.pop();
        if (visited[u])
            continue;
        visited[u] = true;
        for (Link *link : graph->GetEgressLinks(u)) {
            if (link->status == Conflict) {
                continue;
            }
            NodeId v = link->ter_id;
            if (visited[v])
                continue;
            double weight = f(link);
            if (min_cost_from_src->at(v) > weight_u_to_dst + weight) {
                (*min_cost_from_src)[v] = weight_u_to_dst + weight;
                heap.push(NodeCostPair(v, min_cost_from_src->at(v)));
            }
        }
    }
}

}  // namespace

void AStar::InitWithDst(
    NodeId dst, const double* min_cost_to_dst) {
    dst_ = dst;
    if (min_cost_to_dst) {
        for (int i = 0; i <= max_node_id_; ++i) {
            min_cost_to_dst_[i] = min_cost_to_dst[i];
        }
        return;
    }
    for (int i = 0; i <= max_node_id_; ++i) {
        min_cost_to_dst_[i] = kMaxValue;
    }
    min_cost_to_dst_[dst_] = 0;
    std::priority_queue<NodeCostPair, std::vector<NodeCostPair>> heap;
    std::vector<bool> visited(max_node_id_ + 1, false);
    heap.push(NodeCostPair(dst_, 0));
    while (!heap.empty()) {
        NodeId u = heap.top().id;
        double weight_u_to_dst = heap.top().cost;
        heap.pop();
        if (visited[u])
            continue;
        visited[u] = true;
        for (Link *link : graph_->GetIngressLinks(u)) {
            if (link->status == Conflict) {
                continue;
            }
            NodeId v = link->source_id;
            if (visited[v])
                continue;
            double weight = cost_func_(link);
            if (min_cost_to_dst_[v] > weight_u_to_dst + weight) {
                min_cost_to_dst_[v] = weight_u_to_dst + weight;
                heap.push(NodeCostPair(v, min_cost_to_dst_[v]));
            }
        }
    }
}

double AStar::FindPathFromSrc(
    NodeId src, std::vector<Link*>* reverse_links) {
    if (min_cost_to_dst_[src] == kMaxValue) {
        return kMaxValue;
    }
    std::priority_queue<NodeCostPair, std::vector<NodeCostPair>> heap;
    std::vector<bool> visited(max_node_id_ + 1, false);
    std::vector<double> min_cost(max_node_id_ + 1, kMaxValue);
    std::vector<Link*> node_to_parent_map(max_node_id_ + 1, nullptr);
    heap.push(NodeCostPair(src, min_cost_to_dst_[src]));
    min_cost[src] = min_cost_to_dst_[src];
    // int num_iterations = 0;
    while (!heap.empty()) {
        // ++num_iterations;
        NodeId u = heap.top().id;
        double u_cost = heap.top().cost;
        heap.pop();
        if (u == dst_) {  // Find the shortest path
            // std::cout << "Total iterations: " << num_iterations << "\n";
            Link* cur = node_to_parent_map[dst_];
            while (cur) {
                reverse_links->push_back(cur);
                cur = node_to_parent_map[cur->source_id];
            }
            return u_cost;
        }
        if (visited[u])
            continue;
        visited[u] = true;
        double weight_u_from_src = u_cost - min_cost_to_dst_[u];
        for (Link *link : graph_->GetEgressLinks(u)) {
            if (link->status == Conflict) {
                continue;
            }
            NodeId v = link->ter_id;
            if (visited[v])
                continue;
            double weight = cost_func_(link);
            if (min_cost[v] >
                weight_u_from_src + weight + min_cost_to_dst_[v]) {
                min_cost[v] =
                    weight_u_from_src + weight + min_cost_to_dst_[v];
                node_to_parent_map[v] = link;
                heap.push(NodeCostPair(v, min_cost[v]));
            }
        }
    }
    // std::cout << "Total iterations: " << num_iterations << "\n";
    return kMaxValue;
}

void MulAStar::InitWithDst(
    NodeId dst, const double *min_cost_to_dst)
{
    dst_ = dst;
    if (min_cost_to_dst)
    {
        for (int i = 0; i <= max_node_id_; ++i)
        {
            min_cost_to_dst_[i] = min_cost_to_dst[i];
        }
        return;
    }
    for (int i = 0; i <= max_node_id_; ++i)
    {
        min_cost_to_dst_[i] = kMaxValue;
    }
    min_cost_to_dst_[dst_] = 0;
    std::priority_queue<NodeCostPair, std::vector<NodeCostPair>> heap;
    std::vector<bool> visited(max_node_id_ + 1, false);
    heap.push(NodeCostPair(dst_, 0));
    while (!heap.empty())
    {
        NodeId u = heap.top().id;
        double weight_u_to_dst = heap.top().cost;
        heap.pop();
        if (visited[u])
            continue;
        visited[u] = true;
        for (Link *link : graph_->GetMulIngressLinks(u))
        {
            if (link->status == Conflict)
            {
                continue;
            }
            NodeId v = link->source_id;
            if (visited[v])
                continue;
            double weight = cost_func_(link);
            if (min_cost_to_dst_[v] > weight_u_to_dst + weight)
            {
                min_cost_to_dst_[v] = weight_u_to_dst + weight;
                heap.push(NodeCostPair(v, min_cost_to_dst_[v]));
            }
        }
    }
}

double Dijkstra::FindPathFromSrc(
    NodeId src, std::vector<Link*>* reverse_links) {
    std::priority_queue<NodeCostPair, std::vector<NodeCostPair>> heap;
    std::vector<bool> visited(max_node_id_ + 1, false);
    std::vector<Link*> node_to_parent_map(max_node_id_ + 1, nullptr);
    std::vector<double> cost_from_src(max_node_id_ + 1, kMaxValue);
    heap.push(NodeCostPair(src, 0));
    cost_from_src[src] = 0;
    // int num_iterations = 0;
    while (!heap.empty()) {
        // ++num_iterations;
        NodeId u = heap.top().id;
        double weight_u_from_src = heap.top().cost;
        heap.pop();
        if (u == dst_) {  // Find the shortest path
            // std::cout << "Total iterations: " << num_iterations << "\n";
            Link* cur = node_to_parent_map[dst_];
            while (cur) {
                reverse_links->push_back(cur);
                cur = node_to_parent_map[cur->source_id];
            }
            return weight_u_from_src;
        }
        if (visited[u])
            continue;
        visited[u] = true;
        for (Link *link : graph_->GetEgressLinks(u)) {
            if (link->status == Conflict) {
                continue;
            }
            NodeId v = link->ter_id;
            if (visited[v])
                continue;
            double weight = cost_func_(link);
            if (cost_from_src[v] > weight_u_from_src + weight) {
                cost_from_src[v] = weight_u_from_src + weight;
                node_to_parent_map[v] = link;
                heap.push(NodeCostPair(v, cost_from_src[v]));
            }
        }
    }
    // std::cout << "Total iterations: " << num_iterations << "\n";
    return kMaxValue;
}

const std::vector<VisitInfo>* BackwardDfs::GetReachabilityInfo() {
    std::vector<double> min_delay_from_src(
        graph_->GetMaxNodeId() + 1, kMaxValue);
    ComputeCostFromSrc(graph_, flow_.from, LinkDelay, &min_delay_from_src);
    // Start DFS search
    std::vector<Snapshot> back_stack;
    back_stack.reserve(1000000);
    // Create a fake link with delay = 0
    Link fake_link(-1, flow_.to, -1, 0, 0, 0, 0);
    Snapshot snapshot;
    snapshot.link = &fake_link;
    snapshot.stage = 0;
    back_stack.push_back(snapshot);
    // Track the searched path
    double path_delay = 0;
    double path_cost = 0;
    // Start BP search
    while (!back_stack.empty()) {
        Snapshot snap = back_stack.back();
        back_stack.pop_back();
        Link* back_link = snap.link;
        NodeId u = back_link->source_id;
        if (snap.stage == 1) {
            path_delay -= back_link->delay;
            path_cost -= back_link->cost;
        } else {
            double new_delay = path_delay + back_link->delay;
            double new_cost = path_cost + back_link->cost;
            if (u == flow_.from) {
                if (flow_.CheckDelayUb(new_delay) &&
                    flow_.CheckDelayLb(new_delay)) {
                    visit_info_[u].CheckDominanceAndUpdate(
                        new_delay, new_cost);
                }
            } else {
                // prune
                if (flow_.CheckDelayUb(min_delay_from_src[u] + new_delay) &&
                    visit_info_[u].CheckDominanceAndUpdate(
                        new_delay, new_cost)) {
                    const std::vector<Link *> &ingress_links =
                        graph_->GetIngressLinks(u);
                    if (!ingress_links.empty()) {
                        path_delay = new_delay;
                        path_cost = new_cost;
                        snap.stage = 1;
                        back_stack.push_back(snap);
                        for (Link *link : ingress_links) {
                            if (link->status == Conflict ||
                                link->source_id == flow_.to) {
                                continue;
                            }
                            Snapshot new_snap;
                            new_snap.stage = 0;
                            new_snap.link = link;
                            back_stack.push_back(new_snap);
                        }
                    }
                }
            }
        }
    }
    for (VisitInfo& info : visit_info_) {
        info.Process();
    }
    return &visit_info_;
}

const std::vector<QuickVisitInfo>* ReachabilityInfo::GetReachabilityInfo() {
    std::vector<double> min_delay_from_src(
        graph_->GetMaxNodeId() + 1, kMaxValue);
    ComputeCostFromSrc(graph_, flow_.from, LinkDelay, &min_delay_from_src);
    std::priority_queue<NodeDelayCostTriple,
                        std::vector<NodeDelayCostTriple>> heap;
    heap.push(NodeDelayCostTriple(flow_.to, 0, 0));
    while (!heap.empty()) {
        NodeId u = heap.top().id;
        double cost_u_to_dst = heap.top().cost;
        double delay_u_to_dst = heap.top().delay;
        heap.pop();
        if (!visit_info_[u].FastCheckDominanceAndUpdate(
            delay_u_to_dst, cost_u_to_dst)) {
            continue;
        }
        for (Link *link : graph_->GetIngressLinks(u)) {
            if (link->status == Conflict) {
                continue;
            }
            NodeId v = link->source_id;
            if (v == flow_.to)
                continue;
            double cost_v_to_dst = cost_u_to_dst + link->cost;
            double delay_v_to_dst = delay_u_to_dst + link->delay;
            if (min_delay_from_src.at(v) + delay_v_to_dst <= flow_.delay_ub) {
                heap.push(NodeDelayCostTriple(
                    v, delay_v_to_dst, cost_v_to_dst));
            }
        }
    }
    for (QuickVisitInfo& info : visit_info_) {
        info.Process();
    }
    // 开启新的排序对于pulse搜索帮助似乎不大
    // for (NodeId u = 0; u < visit_info_.size(); ++u) {
    //     const std::vector<Link*> all_egress_links = graph_->GetEgressLinks(u);
    //     visit_info_[u].SortEgressLinksBasedOnDelayBudget(
    //         all_egress_links, visit_info_);
    // }
    return &visit_info_;
}

double KShortestPath::Init(NodeId src, NodeId dst) {
    src_ = src;
    dst_ = dst;
    all_paths_.clear();
    path_heap_ = std::priority_queue<PathCostPair>();
    astar_.InitWithDst(dst_);
    std::vector<Link*> shortest_path_reverse;
    shortest_path_reverse.reserve(50);
    double cost =
        astar_.FindPathFromSrc(src_, &shortest_path_reverse);
    all_paths_.emplace_back(shortest_path_reverse);
    PathCostPair path_cost_pair;
    path_cost_pair.cost = cost;
    path_cost_pair.path_ptr = &all_paths_.back();
    path_heap_.push(path_cost_pair);
    return cost;
}

void KShortestPath::AddConflictSet(const ConflictSet& conflict_set) {
    conflict_sets_.push_back(conflict_set);
    ConflictStatus status(&conflict_sets_.back());
    for (ConditionalPath& path : all_paths_) {
        if (!path.valid) {
            continue;
        }
        status.Reset();
        status.Init(path.included_links);
        if (status.CoverConflictSet()) {
            path.valid = false;
        }
    }
}

double KShortestPath::FindNextPath() {
    while (!path_heap_.top().path_ptr->valid) {
        path_heap_.pop();
    }
    ConditionalPath* pre_path = path_heap_.top().path_ptr;
    path_heap_.pop();
    std::vector<Link*> included_links = pre_path->included_links;
    std::vector<Link*> excluded_links = pre_path->excluded_links;
    const std::vector<Link*>& links = pre_path->links;
    // Mark conflict links
    for (Link* link : included_links) {
        for (Link* ingress_link : graph_->GetIngressLinks(link->source_id)) {
            ingress_link->status = Conflict;
        }
    }
    for (Link* link : excluded_links) {
        link->status = Conflict;
    }
    // Init Conflict Status
    std::vector<ConflictStatus> conflict_status;
    conflict_status.reserve(conflict_sets_.size());
    for (ConflictSet& conflict_set : conflict_sets_) {
        conflict_status.emplace_back(&conflict_set);
        conflict_status.back().Init(included_links);
    }
    // Start KSP search
    double base_cost = 0;
    for (Link* link : included_links) {
        base_cost += cost_func_(link);
    }
    double cost2 = 0;
    if (cost_func2_) {
        for (Link* link : included_links) {
            cost2 += cost_func2_(link);
        }
        std::vector<Link*> dumb_path;
        dumb_path.reserve(50);
        cost2 += astar2_->FindPathFromSrc(
            links[included_links.size()]->source_id, &dumb_path);
    }
    if (cost2 <= cost2_ub_) {
        for (int k = included_links.size(); k < links.size(); ++k) {
            Link* cur_link = links[k];
            excluded_links.push_back(cur_link);
            cur_link->status = Conflict;
            std::vector<Link*> reverse_path;
            reverse_path.reserve(50);
            NodeId mid_node_id = cur_link->source_id;
            double cost = astar_.FindPathFromSrc(mid_node_id, &reverse_path);
            if (cost < kMaxValue) {
                all_paths_.emplace_back(
                    reverse_path, included_links, excluded_links);
                PathCostPair path_cost_pair;
                path_cost_pair.cost = cost + base_cost;
                path_cost_pair.path_ptr = &all_paths_.back();
                path_heap_.push(path_cost_pair);
            }
            // Prepare for the next search
            if (k == links.size() - 1) {
                break;
            }
            bool stop = false;
            for (ConflictStatus& cs : conflict_status) {
                cs.Add(cur_link);
                if (cs.CoverConflictSet()) {
                    stop = true;
                    break;
                }
            }
            if (stop) {
                break;
            }
            included_links.push_back(cur_link);
            for (Link* ingress_link : graph_->GetIngressLinks(mid_node_id)) {
                ingress_link->status = Conflict;
            }
            base_cost += cost_func_(cur_link);
            excluded_links.pop_back();
        }
    }
    // Reset link status.
    for (Link* link : links) {
        for (Link* ingress_link : graph_->GetIngressLinks(link->source_id)) {
            ingress_link->status = link_status_backup_[ingress_link->link_id];
        }
    }
    links.back()->status = Available;
    for (Link* link : excluded_links) {
        link->status = Available;
    }
    // Return cost
    if (!path_heap_.empty()) {
        return path_heap_.top().cost;
    }
    return kMaxValue;
}

double ComputeCost(const std::vector<Link*>& links) {
    double cost = 0;
    for (Link* link : links) {
        cost += link->cost;
    }
    return cost;
}

double ComputeDelay(const std::vector<Link*>& links) {
    double delay = 0;
    for (Link* link : links) {
        delay += link->delay;
    }
    return delay;
}

double CalculateMultiplier(Graph* graph, const Flow &flow) {
    Dijkstra delay_sp_(graph, LinkDelay);
    delay_sp_.InitWithDst(flow.to);
    std::vector<Link*> links;
    links.reserve(100);
    double min_delay = delay_sp_.FindPathFromSrc(flow.from, &links);
    double min_delay_path_cost = ComputeCost(links);
    // std::cout << "min delay path cost : " << min_delay_path_cost << "\n";
    if (min_delay > flow.delay_ub) {
        return 0;
    }
    Dijkstra cost_sp_(graph, LinkCost);
    cost_sp_.InitWithDst(flow.to);
    links.clear();
    double min_cost = cost_sp_.FindPathFromSrc(flow.from, &links);
    // std::cout << "min cost: " << min_cost << "\n";
    double min_cost_path_delay = ComputeDelay(links);
    if (min_cost_path_delay < flow.delay_lb) {
        double min_cost_delay_ratio = kMaxValue;
        for (Link& link : graph->GetMutableLinks()) {
            double ratio = link.cost / link.delay;
            if (ratio < min_cost_delay_ratio) {
                min_cost_delay_ratio = ratio;
            }
        }
        auto weight_func = [min_cost_delay_ratio](Link* link) {
            return link->cost - min_cost_delay_ratio * link->delay;
        };
        Dijkstra combined_sp(graph, weight_func);
        combined_sp.InitWithDst(flow.to);
        links.clear();
        double min_weight = combined_sp.FindPathFromSrc(flow.from, &links);
        double delay = ComputeDelay(links);
        if (delay < flow.delay_lb) {
            // std::cout << "case 1a\n";
            return -min_cost_delay_ratio;
        } else {
            double cost = ComputeCost(links);
            double multiplier =
                (cost - min_cost) / (min_cost_path_delay - delay);
            assert(multiplier >= -min_cost_delay_ratio);
            // std::cout << "case 1b " << multiplier << "\n";
            return multiplier;
        }
    } else if (min_cost_path_delay > flow.delay_ub) {
        // std::cout << "case 2\n";
        return (min_delay_path_cost - min_cost) / (min_cost_path_delay - min_delay);
        // double multiplier;
        // double left_intercept = min_cost;
        // double left_slop = min_cost_path_delay;
        // double right_intercept = min_delay_path_cost;
        // double right_slop = min_delay;
        // double derivative;
        // double last = -1;
        // while (true) {
        //     // std::cout << "left: " << left_intercept << " + " << left_slop - flow.delay_ub << "\n";
        //     // std::cout << "right: " << right_intercept << " + " << right_slop - flow.delay_ub << "\n";
        //     multiplier = (right_intercept - left_intercept) / (left_slop - right_slop);
        //     if (multiplier == last) {
        //         return multiplier;
        //     }
        //     auto weight_func = [multiplier](Link* link) {
        //         if (multiplier < 100) {
        //             return link->cost + multiplier * link->delay;
        //         } else {
        //             return link->cost / multiplier + link->delay;
        //         }
        //     };
        //     Dijkstra combined_sp(graph, weight_func);
        //     combined_sp.InitWithDst(flow.to);
        //     links.clear();
        //     double min_weight = combined_sp.FindPathFromSrc(flow.from, &links);
        //     double delay = ComputeDelay(links);
        //     derivative = delay - flow.delay_ub;
        //     std::cout << multiplier << " " << derivative << "\n";
        //     if (derivative > 1e-3) {
        //         left_intercept = ComputeCost(links);
        //         left_slop = delay;
        //     } else if (derivative < -1e-3) {
        //         right_intercept = ComputeCost(links);
        //         right_slop = delay;
        //     } else {
        //         return multiplier;
        //     }
        //     last = multiplier;
        // }
    } else {
        // std::cout << "case 3\n";
        return 0;
    }
}


void GetOriginalPath(Graph* graph, const std::vector<Link *> &path, const Flow &flow, std::vector<Link*> &original_path)
{
    original_path.clear();
    original_path.reserve(path.size());
    for(int i = 0; i < path.size();i++)
    {
        if(path[i]->link_id >= (graph->GetMaxLinkId()+1) * (flow.SF_number+1)){
            continue;
        }else{
            original_path.push_back(new Link((path[i]->link_id)%(graph->GetMaxLinkId()+1),
                    (path[i]->source_id)%(graph->GetMaxNodeId()+1),
                    (path[i]->ter_id)%(graph->GetMaxNodeId()+1),
                    path[i]->cost,
                    path[i]->bandwidth,
                    path[i]->delay,
                    path[i]->srlg_num));
        }
    }
}

void GetUsedNodes(Graph* graph, const std::vector<Link*> &path, const Flow &flow, std::vector<NodeInfo> &used_nodes)
{
    used_nodes.clear();
    used_nodes.reserve(flow.SF_number);
    for(int i =0; i < path.size();i++)
    {
        if(path[i]->link_id >= (graph->GetMaxLinkId()+1) * (flow.SF_number+1)){
            NodeId original_nodeid = path[i]->source_id % (graph->GetMaxNodeId()+1);
            used_nodes.push_back(graph->GetNodeInfo(original_nodeid));
        }
    }
}
