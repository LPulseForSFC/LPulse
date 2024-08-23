#include "ra_ra.h"
#include "shortest_path.h"

double MSG::ComputeBandwidth(const std::vector<Link*>& links)
{
    if (links.empty())
    {
        return 0;
    }
    double bandwidth = links.back()->bandwidth;
    for (Link* link : links)
    {
        if (link->bandwidth < bandwidth)
        {
            bandwidth = link->bandwidth;
        }
    }
    return bandwidth;
}

void MSG::LFG(const Flow &flow, CostFunc f)
{
    int SF_number = flow.SF_number;
    std::vector<std::vector<int>> step_nodes;
    step_nodes.reserve(SF_number + 2);
    for (int i = 0; i < SF_number + 2; i++)
    {
        step_nodes.push_back(std::vector<int>());
    }
    step_nodes[0].push_back(flow.from);
    step_nodes[SF_number + 1].push_back(flow.to);
    for (int i = 0; i < SF_number; i++)
    {
        for (NodeId node : flow.Subset_sequence[i])
        {
            step_nodes[i + 1].push_back(node);
        }
    }
    int NodeID = 0;
    node_g_to_lfg.reserve(SF_number + 2);
    for (int i = 0; i < SF_number + 2; i++)
    {
        node_g_to_lfg.push_back(std::unordered_map<NodeId, NodeId>());
        for (NodeId node : step_nodes[i])
        {
            node_lfg_to_g[NodeID] = node;
            node_g_to_lfg[i][node] = NodeID;
            NodeID++;
        }
    }
    AStar a_star_cost(graph_, f);
    int LinkID = 0;
    std::vector<Link> step_links;
    step_links.reserve(10000);
    link_to_path_.reserve(10000);
    for (int i = 0; i < SF_number + 1; i++)
    {
        for (NodeId source_node : step_nodes[i])
        {
            for (NodeId dst_node : step_nodes[i + 1])
            {
                // Link(LinkId link_id_in, NodeId source_id_in, NodeId ter_id_in,
                // double cost_in, double bandwidth_in, double delay_in, int srlg_num_in)
                double source_node_cost = 0 ? i == 0 : graph_->GetNodeInfo(source_node).cost;
                double dst_node_cost = 0 ? i == SF_number + 1 : graph_->GetNodeInfo(dst_node).cost;
                double source_node_delay = 0 ? i == 0 : graph_->GetNodeInfo(source_node).delay;
                double dst_node_delay = 0 ? i == SF_number + 1: graph_->GetNodeInfo(dst_node).delay;
                a_star_cost.InitWithDst(dst_node);
                std::vector<Link*> reverse_links;
                double cost = a_star_cost.FindPathFromSrc(source_node, &reverse_links);
                link_to_path_.push_back(reverse_links);
                double delay = ComputeDelay(reverse_links);
                double bandwidth = ComputeBandwidth(reverse_links);
                NodeId src = node_g_to_lfg[i][source_node];
                NodeId dst = node_g_to_lfg[i + 1][dst_node];
                step_links.emplace_back(LinkID, src, dst, cost + (source_node_cost + dst_node_cost)/2
                                        , bandwidth, delay + (source_node_delay + dst_node_delay)/2, 0);
                LinkID++;
            }
        }
    }
    LFG_ = new Graph(step_links);
    LFG_flow_ = flow;
    LFG_flow_.from =  node_g_to_lfg[0][flow.from];
    LFG_flow_.to =  node_g_to_lfg[SF_number + 1][flow.to]; 
}

const std::vector<Link*> MSG::TransformPath(const std::vector<Link*>& links)
{
    std::vector<Link*> originl_path;
    for (Link* step_link : links)
    {
        const std::vector<Link*> &reverse_path = link_to_path_[step_link->link_id];
        for(std::vector<Link*>:: const_reverse_iterator it = reverse_path.rbegin(); it != reverse_path.rend(); ++it)
        {
            originl_path.push_back(*it);
        }
    }
    return originl_path;
}

std::vector<NodeInfo> MSG::GetUsedNodes(const std::vector<Link*>& links)
{
    std::vector<NodeInfo> used_nodes;
    for (int i = 0; i < links.size()-1; i++)
    {
        NodeId node = node_lfg_to_g[links[i]->ter_id];
        used_nodes.push_back(graph_->GetNodeInfo(node));
    }
    return used_nodes;
}

Path RA::FindPath(const Flow &flow)
{
    clock_t start_time;
    clock_t end_time;
    start_time = clock();
    LFG(flow);
    KShortestPath ksp(LFG_, LinkCost);
    double cost = ksp.Init(LFG_flow_.from, LFG_flow_.to);
    ap_info_.iteration_num = 1;
    Path result;
    const std::vector<Link *> &cur_path = ksp.GetPath();
    result.path_link = TransformPath(cur_path);
    result.path_used_function_nodes = GetUsedNodes(cur_path);    
    result.CompletePathwithNodeWeight();
    while (cost < kMaxValue)
    {
        const std::vector<Link *> &cur_path = ksp.GetPath();
        double delay = ComputeDelay(cur_path);
        if (LFG_flow_.CheckDelayUb(delay) && LFG_flow_.CheckDelayLb(delay))
        // if (LFG_flow_.CheckDelayLb(delay))
        {
            result.path_link = TransformPath(cur_path);
            result.path_used_function_nodes = GetUsedNodes(cur_path);
            result.CompletePathwithNodeWeight();
            break;
        }
        ++ap_info_.iteration_num;
        cost = ksp.FindNextPath();
    }
    end_time = clock();
    ap_info_.total_time = end_time - start_time;
    ap_path_ = &result;
    return result;
}

Path DelayRA::FindPath(const Flow &flow)
{
    clock_t start_time;
    clock_t end_time;
    start_time = clock();
    LFG(flow, LinkDelay);
    KShortestPath ksp(LFG_, LinkDelay);
    double cost = ksp.Init(LFG_flow_.from, LFG_flow_.to);
    ap_info_.iteration_num = 1;
    Path result;
    const std::vector<Link *> &cur_path = ksp.GetPath();
    result.path_link = TransformPath(cur_path);
    result.path_used_function_nodes = GetUsedNodes(cur_path);    
    result.CompletePathwithNodeWeight();
    double best_cost = kMaxValue;
    while (cost < kMaxValue)
    {
        const std::vector<Link *> &cur_path = ksp.GetPath();
        double delay = ComputeDelay(cur_path);
        if (LFG_flow_.CheckDelayLb(delay))
        {
            if (LFG_flow_.CheckDelayUb(delay))
            // if (LFG_flow_.CheckDelayLb(delay))
            {
                if (cost < best_cost)
                {
                    result.path_link = TransformPath(cur_path);
                    result.path_used_function_nodes = GetUsedNodes(cur_path);
                    result.CompletePathwithNodeWeight();
                    cost = best_cost;
                }
            }
            else
            {
                break;
            }
        }
        ++ap_info_.iteration_num;
        cost = ksp.FindNextPath();
    }
    end_time = clock();
    ap_info_.total_time = end_time - start_time;
    ap_path_ = &result;
    return result;
}