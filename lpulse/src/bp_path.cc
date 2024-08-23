#include "bp_path.h"
#include "shortest_path.h"

namespace
{

    void SetLinkStatus(const std::vector<Link *> &ap_path,
                       DisjointType type, LinkStatus status, Graph *graph)
    {
        if (type == LinkDisjoint)
        {
            for (Link *link : ap_path)
            {
                if (link->link_id >= 0)
                {
                    link->status = status;
                }
            }
        }
        else if (type == SrlgDisjoint)
        {
            for (Link *link : ap_path)
            {
                if (link->link_id >= 0)
                {
                    assert(link->srlgs.size() > 0);
                    for (int srlg_id : link->srlgs)
                    {
                        graph->ChangeLinkStatus(srlg_id, status);
                    }
                }
            }
        }
    }

    bool CheckDisjoint(const std::vector<Link *> &bp_path,
                       const std::vector<bool> &used_srlg_id_by_ap,
                       const Graph *graph, int *conflict_srlg_id)
    {
        int max_group_size = 0;
        for (int i = 1; i < bp_path.size(); ++i)
        {
            Link *link = bp_path[i];
            if (link->status != Available)
            {
                for (int srlg_id : link->srlgs)
                {
                    if (used_srlg_id_by_ap[srlg_id])
                    {
                        int group_size = graph->GetSrlgGroupSize(srlg_id);
                        if (group_size > max_group_size)
                        {
                            *conflict_srlg_id = srlg_id;
                            max_group_size = group_size;
                        }
                    }
                }
                break;
            }
        }
        return (max_group_size == 0);
    }

} // namespace

double DisjointBp::FindBpPath(
    const std::vector<Link *> &ap_path, const Flow &flow, int &iteration_num)
{
    SetLinkStatus(ap_path, flow.type, Conflict, graph_);
    AStar astar(graph_, LinkDelay);
    astar.InitWithDst(flow.to);
    const double *min_delay_to_dst = astar.GetCostVector();
    for (Link &link : graph_->GetMutableLinks())
    {
        if (link.status == Available)
        {
            link.weight = link.delay + min_delay_to_dst[link.ter_id];
        }
        else
        {
            link.weight = 0;
        }
    }
    graph_->UpdateBpEgressLinks();

    // Start DFS search
    std::vector<Snapshot> bp_stack;
    bp_stack.reserve(1000000);
    // Create a fake link with delay = 0
    Link fake_link(-1, -1, flow.from, 0, 0, 0, 0);
    Snapshot snapshot;
    snapshot.link = &fake_link;
    snapshot.stage = 0;
    bp_stack.push_back(snapshot);
    // Track the searched path
    std::vector<Link *> bp_path;
    double bp_path_delay = 0;
    bp_path.reserve(100);
    std::vector<bool> bp_visited(graph_->GetMaxNodeId() + 1, false);
    // Start BP search
    while (!bp_stack.empty())
    {
        Snapshot bp_snap = bp_stack.back();
        bp_stack.pop_back();
        Link *bp_link = bp_snap.link;
        NodeId u = bp_link->ter_id;
        if (bp_snap.stage == 1)
        {
            bp_path.pop_back();
            bp_path_delay -= bp_link->delay;
            bp_visited[u] = false;
        }
        else
        {
            double new_delay = bp_path_delay + bp_link->delay;
            if (u == flow.to)
            {
                if (flow.CheckDelayUb(new_delay) &&
                    flow.CheckDelayLb(new_delay))
                {
                    // 注意bp_path的第一跳是fake_link
                    path_.clear();
                    ++iteration_num;
                    path_.reserve(bp_path.size());
                    for (int i = 1; i < bp_path.size(); ++i)
                    {
                        path_.push_back(bp_path[i]);
                    }
                    path_.push_back(bp_link);
                    // Reset link status
                    SetLinkStatus(ap_path, flow.type, Available, graph_);
                    return new_delay;
                }
            }
            else
            {
                // prune
                if (flow.CheckDelayUb(min_delay_to_dst[u] + new_delay))
                {
                    const std::vector<Link *> &egress_links =
                        graph_->GetBpEgressLinks(u);
                    if (!egress_links.empty())
                    {
                        bp_path.push_back(bp_link);
                        bp_path_delay += bp_link->delay;
                        bp_visited[u] = true;
                        bp_snap.stage = 1;
                        bp_stack.push_back(bp_snap);
                        for (Link *link : egress_links)
                        {
                            if (link->status == Conflict)
                            {
                                continue;
                            }
                            NodeId node_id = link->ter_id;
                            if (bp_visited[node_id])
                            {
                                continue;
                            }
                            Snapshot snap;
                            snap.stage = 0;
                            snap.link = link;
                            bp_stack.push_back(snap);
                        }
                    }
                }
            }
        }
    }
    // Reset link status
    SetLinkStatus(ap_path, flow.type, Available, graph_);
    return kMaxValue;
}

double SrlgDisjointBp::FindBpPath(
    const std::vector<Link *> &ap_path, const Flow &flow, int &iteration_num)
{
    astar_.InitWithDst(flow.to, base_min_delay_to_dst_);
    const double *min_delay_to_dst = astar_.GetCostVector();
    // assert(flow.type == SrlgDisjoint);
    // Since link disjoint is a special case of srlg disjoint,
    // SrlgDisjointBp can be also used for the link disjoint cases.
    SetLinkStatus(ap_path, SrlgDisjoint, Affected, graph_);
    for (Link &link : graph_->GetMutableLinks())
    {
        if (link.status == Available)
        {
            link.weight = link.delay + min_delay_to_dst[link.ter_id];
        }
        else
        {
            link.weight = 0;
        }
    }
    graph_->UpdateBpEgressLinks();
    // 记录下ap_path占用了哪些srlg
    std::vector<bool> used_srlg_id_by_ap(graph_->GetMaxSrlgId() + 1, false);
    for (Link *link : ap_path)
    {
        for (int srlg_id : link->srlgs)
        {
            used_srlg_id_by_ap[srlg_id] = true;
        }
    }
    // Start DFS search
    std::vector<Snapshot> bp_stack;
    bp_stack.reserve(1000000);
    // Create a fake link with delay = 0
    Link fake_link(-1, -1, flow.from, 0, 0, 0, 0);
    Snapshot snapshot;
    snapshot.link = &fake_link;
    snapshot.stage = 0;
    bp_stack.push_back(snapshot);
    // Track the searched path
    std::vector<Link *> bp_path;
    double bp_path_delay = 0;
    bp_path.reserve(100);
    std::vector<bool> bp_visited(graph_->GetMaxNodeId() + 1, false);
    int num_conflict_links = 0;
    conflict_set_.Reset();
    // Initialize visit_cnt_
    visit_cnt_.clear();
    visit_cnt_.reserve(graph_->GetMaxNodeId() + 1);
    for (int i = 0; i <= graph_->GetMaxNodeId(); ++i)
    {
        visit_cnt_.emplace_back(flow.delay_ub + 1, false);
    }
    // Start BP search with dominance check
    while (!bp_stack.empty())
    {
        Snapshot bp_snap = bp_stack.back();
        bp_stack.pop_back();
        Link *bp_link = bp_snap.link;
        NodeId u = bp_link->ter_id;
        if (bp_snap.stage == 1)
        {
            bp_path.pop_back();
            bp_path_delay -= bp_link->delay;
            bp_visited[u] = false;
            if (bp_link->status == Conflict)
            {
                --num_conflict_links;
            }
        }
        else
        {
            if (num_conflict_links > 0 || bp_link->status == Conflict)
            {
                continue;
            }
            double new_delay = bp_path_delay + bp_link->delay;
            if (u == flow.to)
            {
                if (flow.CheckDelayUb(new_delay) &&
                    flow.CheckDelayLb(new_delay))
                {
                    ++iteration_num;
                    int conflict_srlg_id;
                    bp_path.push_back(bp_link);
                    bool disjoint = CheckDisjoint(
                        bp_path, used_srlg_id_by_ap,
                        graph_, &conflict_srlg_id);
                    if (disjoint)
                    {
                        // 注意bp_path的第一跳是fake_link
                        path_.clear();
                        path_.reserve(bp_path.size() - 1);
                        for (int i = 1; i < bp_path.size(); ++i)
                        {
                            path_.push_back(bp_path[i]);
                        }
                        // Reset link status
                        SetLinkStatus(ap_path, SrlgDisjoint,
                                      Available, graph_);
                        return new_delay;
                    }
                    else
                    {
                        graph_->ChangeLinkStatus(conflict_srlg_id, Conflict);
                        conflict_set_.MarkConflict(conflict_srlg_id);
                        astar_.InitWithDst(flow.to); // Update distance to dst
                        bp_path.pop_back();
                        for (Link *link : bp_path)
                        {
                            if (link->status == Conflict)
                            {
                                ++num_conflict_links;
                            }
                        }
                    }
                }
            }
            else
            {
                // prune
                if (flow.CheckDelayUb(min_delay_to_dst[u] + new_delay) &&
                    !visit_cnt_[u][new_delay])
                {
                    visit_cnt_[u][new_delay] = true;
                    const std::vector<Link *> &egress_links =
                        graph_->GetBpEgressLinks(u);
                    if (!egress_links.empty())
                    {
                        bp_path.push_back(bp_link);
                        bp_path_delay += bp_link->delay;
                        bp_visited[u] = true;
                        bp_snap.stage = 1;
                        bp_stack.push_back(bp_snap);
                        for (Link *link : egress_links)
                        {
                            if (link->status == Conflict)
                            {
                                continue;
                            }
                            NodeId node_id = link->ter_id;
                            if (bp_visited[node_id])
                            {
                                continue;
                            }
                            Snapshot snap;
                            snap.stage = 0;
                            snap.link = link;
                            bp_stack.push_back(snap);
                        }
                    }
                }
            }
        }
    }
    bp_stack.push_back(snapshot);
    // DFS with dominance check may not generate a correct conflict set.
    // Perform DFS again without dominance check.
    while (!bp_stack.empty())
    {
        Snapshot bp_snap = bp_stack.back();
        bp_stack.pop_back();
        Link *bp_link = bp_snap.link;
        NodeId u = bp_link->ter_id;
        if (bp_snap.stage == 1)
        {
            bp_path.pop_back();
            bp_path_delay -= bp_link->delay;
            bp_visited[u] = false;
            if (bp_link->status == Conflict)
            {
                --num_conflict_links;
            }
        }
        else
        {
            if (num_conflict_links > 0 || bp_link->status == Conflict)
            {
                continue;
            }
            double new_delay = bp_path_delay + bp_link->delay;
            if (u == flow.to)
            {
                if (flow.CheckDelayUb(new_delay) &&
                    flow.CheckDelayLb(new_delay))
                {
                    ++iteration_num;
                    int conflict_srlg_id;
                    bp_path.push_back(bp_link);
                    bool disjoint = CheckDisjoint(
                        bp_path, used_srlg_id_by_ap,
                        graph_, &conflict_srlg_id);
                    if (disjoint)
                    {
                        // 注意bp_path的第一跳是fake_link
                        path_.clear();
                        path_.reserve(bp_path.size() - 1);
                        for (int i = 1; i < bp_path.size(); ++i)
                        {
                            path_.push_back(bp_path[i]);
                        }
                        // Reset link status
                        SetLinkStatus(ap_path, SrlgDisjoint,
                                      Available, graph_);
                        return new_delay;
                    }
                    else
                    {
                        graph_->ChangeLinkStatus(conflict_srlg_id, Conflict);
                        conflict_set_.MarkConflict(conflict_srlg_id);
                        astar_.InitWithDst(flow.to); // Update distance to dst
                        bp_path.pop_back();
                        for (Link *link : bp_path)
                        {
                            if (link->status == Conflict)
                            {
                                ++num_conflict_links;
                            }
                        }
                    }
                }
            }
            else
            {
                // prune
                if (flow.CheckDelayUb(min_delay_to_dst[u] + new_delay))
                {
                    const std::vector<Link *> &egress_links =
                        graph_->GetBpEgressLinks(u);
                    if (!egress_links.empty())
                    {
                        bp_path.push_back(bp_link);
                        bp_path_delay += bp_link->delay;
                        bp_visited[u] = true;
                        bp_snap.stage = 1;
                        bp_stack.push_back(bp_snap);
                        for (Link *link : egress_links)
                        {
                            if (link->status == Conflict)
                            {
                                continue;
                            }
                            NodeId node_id = link->ter_id;
                            if (bp_visited[node_id])
                            {
                                continue;
                            }
                            Snapshot snap;
                            snap.stage = 0;
                            snap.link = link;
                            bp_stack.push_back(snap);
                        }
                    }
                }
            }
        }
    }
    // Reset link status
    SetLinkStatus(ap_path, SrlgDisjoint, Available, graph_);
    return kMaxValue;
}

double KspBp::FindBpPath(
    const std::vector<Link *> &ap_path, const Flow &flow, int &iteration_num)
{
    SetLinkStatus(ap_path, flow.type, Conflict, graph_);
    KShortestPath ksp(graph_, LinkDelay);
    double delay = ksp.Init(flow.from, flow.to);
    ++iteration_num;
    // std::cout << "hello " << delay << " " << flow.delay_lb << std::endl;
    // clock_t start_time = clock();
    while (!flow.CheckDelayLb(delay))
    {
        // std::cout << iteration_num << std::endl;
        ++iteration_num;
        delay = ksp.FindNextPath();
    }
    // std::cout << "Total KSP iterations for BP: " << num_iterations << "\n";
    // Reset link status
    SetLinkStatus(ap_path, flow.type, Available, graph_);
    // clock_t end_time = clock();
    if (flow.CheckDelayUb(delay))
    {
        path_ = ksp.GetPath();
        return delay;
    }
    else
    {
        return kMaxValue;
    }
}
