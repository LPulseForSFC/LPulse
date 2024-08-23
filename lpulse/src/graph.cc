#include <algorithm> // std::random_shuffle
#include <assert.h>
#include <cstdlib> // std::rand, std::srand
#include <fstream>
#include <sstream>
#include <iostream>

#include "graph.h"

namespace
{
    void ReadLinksFromFile(const std::string &file_path,
                           std::vector<Link> *links)
    {
        std::string line;
        std::ifstream in(file_path);
        if (in.fail())
        {
            std::cout << "File not found" << std::endl;
            return;
        }
        int line_cnt = 0;
        links->reserve(10000);
        while (getline(in, line) && in.good())
        {
            ++line_cnt;
            if (line_cnt == 1)
                continue;
            // 把line里的单元格数字字符提取出来，“,”为单元格分隔符
            std::vector<std::string> buffer;
            std::string str;
            for (int i = 0; i < line.size(); ++i)
            {
                if (line[i] == ',')
                {
                    buffer.push_back(str);
                    str.clear();
                    continue;
                }
                str.push_back(line[i]);
            }
            assert(buffer.size() >= 8);
            links->emplace_back(
                atoi(buffer[0].c_str()), atoi(buffer[1].c_str()),
                atoi(buffer[2].c_str()), atof(buffer[4].c_str()),
                atof(buffer[5].c_str()), atof(buffer[6].c_str()),
                atoi(buffer[7].c_str()));
            if (atoi(buffer[7].c_str()) > 0)
            {
                std::string tmp;
                for (int i = 0; i < str.size(); ++i)
                {
                    if (str[i] == '|')
                    {
                        links->back().srlgs.push_back(atoi(tmp.c_str()));
                        tmp.clear();
                        continue;
                    }
                    tmp.push_back(str[i]);
                }
                links->back().srlgs.push_back(atoi(tmp.c_str()));
            }
            // Sort link's srlgs
            std::sort(links->back().srlgs.begin(), links->back().srlgs.end());
        }
        in.close();
    }

    bool Compare(Link *a, Link *b)
    {
        return a->weight > b->weight;
    }

    bool Uncompare(Link *a, Link *b)
    {
        return a->weight_mask > b->weight_mask;
    }

    bool CompareLayer(Link *a, Link *b)
    {
        // return a->weight < b->weight;
        // if(a->weight_mask == b->weight_mask)
        // {
        //     return a->weight < b->weight;
        // }else{
        //     return a->weight_mask < b->weight_mask;
        // }
        if(a->weight_mask != b->weight_mask)
        {
            return a->weight_mask < b->weight_mask;
        }else{
            return a->weight < b->weight;
        }
        // return a->weight < b->weight;
        // return a->weight_mask < b->weight_mask;
    }

} // namespace

void Path::Print() const
{
    if (path_info.empty())
    {
        std::cout << "Empty Path\n";
        return;
    }
    std::cout << "Path:\nNode List: " << path_info[0];
    for (int i = 1; i < path_info.size(); ++i)
    {
        std::cout << "->" << path_info[i];
    }
    if (!path_link.empty())
    {
        std::cout << "\nLink List: ";
        for (int i = 0; i < path_link.size(); ++i)
        {
            if (i > 0)
            {
                std::cout << "->";
            }
            std::cout << path_link[i]->link_id;
            std::vector<int> srlgs = path_link[i]->srlgs;
            if (srlgs.size() > 1)
            {
                std::cout << "(" << srlgs[0];
                for (int j = 1; j < srlgs.size() - 1; ++j)
                {
                    std::cout << "," << srlgs[j];
                }
                std::cout << ")";
            }
        }
        if (path_used_function_nodes.size() > 0)
        {
            std::cout << "\nFunction Node List: ";
            for (int i = 0; i < path_used_function_nodes.size(); ++i)
            {
                if (i > 0)
                {
                    std::cout << "|";
                }
                std::cout << path_used_function_nodes[i].node_id;
            }
        
        }
    }
    std::cout << ".\nCost is: " << cost << ", Delay is: "
              << delay << ".\n";
}

bool Path::Verify() const
{
    if (path_link.size() + 1 != path_info.size())
    {
        return false;
    }
    for (int i = 0; i < path_link.size(); ++i)
    {
        Link *link = path_link[i];
        if (link->source_id != path_info[i])
        {
            return false;
        }
        if (link->ter_id != path_info[i + 1])
        {
            return false;
        }
    }
    return true;
}

std::string Path::ToString() const
{
    std::string result;
    for (int i = 0; i < path_info.size(); ++i)
    {
        result += std::to_string(path_info[i]);
        if (i != path_info.size() - 1)
            result += "->";
    }
    return result;
}

bool PathPair::VerfyLinkDisjoint() const
{
    if (ap_path.Empty() && bp_path.Empty())
    {
        return true;
    }
    if (!ap_path.Verify())
    {
        return false;
    }
    if (!bp_path.Verify())
    {
        return false;
    }
    std::unordered_set<Link *> links;
    if (ap_path.path_info.front() != bp_path.path_info.front())
    {
        return false;
    }
    if (ap_path.path_info.back() != bp_path.path_info.back())
    {
        return false;
    }
    for (Link *link : ap_path.path_link)
    {
        links.insert(link);
    }
    for (Link *link : bp_path.path_link)
    {
        links.insert(link);
    }
    if (links.size() < ap_path.path_link.size() +
                           bp_path.path_link.size())
    {
        return false;
    }
    return true;
}

void Graph::UpdateBpEgressLinks()
{
    bp_node_to_egress_links_ = node_to_egress_links_;
    for (NodeId i = 0; i < size_; ++i)
    {
        std::sort(bp_node_to_egress_links_[i].begin(),
                  bp_node_to_egress_links_[i].end(), Compare);
    }
    // for (int i = 0; i < bp_node_to_egress_links_.size(); ++i) {
    //     const std::vector<Link *>& egress_links = node_to_egress_links_[i];
    //     if (egress_links.size() > 0) {
    //         std::vector<Link *> available_links;
    //         available_links.reserve(egress_links.size());
    //         bp_node_to_egress_links_[i].clear();
    //         for (Link* link : egress_links) {
    //             if (link->status != Available) {
    //                 bp_node_to_egress_links_[i].push_back(link);
    //             } else {
    //                 available_links.push_back(link);
    //             }
    //         }
    //         bp_node_to_egress_links_[i].insert(
    //             bp_node_to_egress_links_[i].end(),
    //             available_links.begin(), available_links.end());
    //     }
    // }
}

Graph::Graph(const std::string &file_path)
{
    srlg_group_.reserve(1000000);
    for (int i = 0; i < 1000000; ++i)
    {
        srlg_group_.push_back({});
    }
    ReadLinksFromFile(file_path, &links_);
    // std::srand(0);
    // std::random_shuffle(links_.begin(), links_.end());
    max_node_id_ = -1;
    max_link_id_ = -1;
    max_srlg_id_ = -1;
    for (Link &link : links_)
    {
        if (link.source_id > max_node_id_)
            max_node_id_ = link.source_id;
        if (link.ter_id > max_node_id_)
            max_node_id_ = link.ter_id;
        if (link.link_id > max_link_id_)
        {
            max_link_id_ = link.link_id;
        }
        // if (link.link_id == 15125) {
        //     link.delay = 100;
        // }
        if (link.srlg_num != 0)
        {
            for (int i = 0; i < link.srlgs.size(); ++i)
            {
                if (link.srlgs[i] > max_srlg_id_)
                {
                    max_srlg_id_ = link.srlgs[i];
                }
                srlg_group_[link.srlgs[i]].push_back(&link);
                // if (link.srlgs[i] == 10) {
                //     link.delay = 6;
                // }
            }
        }
        nodes_.insert(link.source_id);
        nodes_.insert(link.ter_id);
    }
    // 为每个link设置一个专属的srlg，该srlg仅包含一条link.
    for (Link &link : links_)
    {
        link.srlg_num += 1;
        link.srlgs.push_back(++max_srlg_id_);
        srlg_group_[max_srlg_id_].push_back(&link);
    }
    size_ = max_node_id_ + 1;
    node_to_egress_links_.clear();
    node_to_ingress_links_.clear();
    node_to_egress_links_cost_.clear();
    node_to_ingress_links_cost_.clear();
    bp_node_to_egress_links_.clear();
    node_to_egress_links_.reserve(size_);
    node_to_ingress_links_.reserve(size_);
    node_to_egress_links_cost_.reserve(size_);
    node_to_ingress_links_cost_.reserve(size_);
    bp_node_to_egress_links_.reserve(size_);
    for (NodeId i = 0; i < size_; ++i)
    {
        // assert(nodes_.find(i) != nodes_.end());
        node_to_egress_links_.push_back({});
        bp_node_to_egress_links_.push_back({});
        node_to_ingress_links_.push_back({});
    }
    for (Link &link : links_)
    {
        node_to_egress_links_[link.source_id].push_back(&link);
        node_to_ingress_links_[link.ter_id].push_back(&link);
    }
    node_to_egress_links_cost_ = node_to_egress_links_;
    node_to_ingress_links_cost_ = node_to_ingress_links_;
    while (!recent_flows.empty())
    {
        recent_flows.pop();
    }
}

Graph::Graph(const std::string &file_path, const std::string &node_path)
{
    srlg_group_.reserve(1000000);
    for (int i = 0; i < 1000000; ++i)
    {
        srlg_group_.push_back({});
    }
    ReadLinksFromFile(file_path, &links_);
    // std::srand(0);
    // std::random_shuffle(links_.begin(), links_.end());
    max_node_id_ = -1;
    max_link_id_ = -1;
    max_srlg_id_ = -1;
    for (Link &link : links_)
    {
        if (link.source_id > max_node_id_)
            max_node_id_ = link.source_id;
        if (link.ter_id > max_node_id_)
            max_node_id_ = link.ter_id;
        if (link.link_id > max_link_id_)
        {
            max_link_id_ = link.link_id;
        }
        // if (link.link_id == 15125) {
        //     link.delay = 100;
        // }
        if (link.srlg_num != 0)
        {
            for (int i = 0; i < link.srlgs.size(); ++i)
            {
                if (link.srlgs[i] > max_srlg_id_)
                {
                    max_srlg_id_ = link.srlgs[i];
                }
                srlg_group_[link.srlgs[i]].push_back(&link);
                // if (link.srlgs[i] == 10) {
                //     link.delay = 6;
                // }
            }
        }
        nodes_.insert(link.source_id);
        nodes_.insert(link.ter_id);
    }
    // 为每个link设置一个专属的srlg，该srlg仅包含一条link.
    for (Link &link : links_)
    {
        link.srlg_num += 1;
        link.srlgs.push_back(++max_srlg_id_);
        srlg_group_[max_srlg_id_].push_back(&link);
    }
    size_ = max_node_id_ + 1;
    node_to_egress_links_.clear();
    node_to_ingress_links_.clear();
    node_to_egress_links_cost_.clear();
    node_to_ingress_links_cost_.clear();
    bp_node_to_egress_links_.clear();
    node_to_egress_links_.reserve(size_);
    node_to_ingress_links_.reserve(size_);
    node_to_egress_links_cost_.reserve(size_);
    node_to_ingress_links_cost_.reserve(size_);
    bp_node_to_egress_links_.reserve(size_);
    for (NodeId i = 0; i < size_; ++i)
    {
        // assert(nodes_.find(i) != nodes_.end());
        node_to_egress_links_.push_back({});
        bp_node_to_egress_links_.push_back({});
        node_to_ingress_links_.push_back({});
    }
    for (Link &link : links_)
    {
        node_to_egress_links_[link.source_id].push_back(&link);
        node_to_ingress_links_[link.ter_id].push_back(&link);
    }
    node_to_egress_links_cost_ = node_to_egress_links_;
    node_to_ingress_links_cost_ = node_to_ingress_links_;

    std::ifstream in(node_path);
    std::string line;
    std::getline(in, line);
    node_to_weight_.clear();
    node_to_weight_.reserve(size_);
    std::vector<std::vector<int>> node_to_function;
    node_to_function.reserve(size_);
    int max_function_id = 0;
    for (NodeId i = 0; i < size_; ++i)
    {
        node_to_function.push_back({});
    }
    while(std::getline(in,line))
    {
        std::stringstream line_stream(line);
        std::string cell;
        std::vector<std::string> row_data;
        while(std::getline(line_stream,cell,','))
        {
            row_data.push_back(cell);
        }
        node_to_weight_[std::stoi(row_data[0])] = NodeInfo(std::stod(row_data[0]),
                                                        std::stod(row_data[3]), 
                                                        std::stod(row_data[2]), 
                                                        std::stod(row_data[1]));
        if (row_data.size() > 4)
        {
            std::string node_function = row_data.back();
            std::stringstream node_function_stream(node_function);
            std::string function;

            std::vector<int> function_list;
            while(std::getline(node_function_stream,function,'|'))
            {
                int function_id = std::stoi(function);
                max_function_id = std::max(max_function_id,function_id);
                function_list.push_back(function_id);
            }
            node_to_function[std::stoi(row_data[0])] = function_list;
        }
    }
    function_to_node_.clear();
    function_to_node_.reserve(max_function_id+1);
    for(int i = 0; i < max_function_id+1; i++)
    {
        function_to_node_.push_back({});
    }
    for(int i = 0; i < node_to_function.size(); i++)
    {
        for(int j = 0; j < node_to_function[i].size(); j++)
        {
            function_to_node_[node_to_function[i][j]].push_back(i);
        }
    }
    in.close();
}

Graph::Graph(const std::vector<Link> &link)
{
    srlg_group_.reserve(1000000);
    for (int i = 0; i < 1000000; ++i)
    {
        srlg_group_.push_back({});
    }
    links_ = link;
    // std::srand(0);
    // std::random_shuffle(links_.begin(), links_.end());
    max_node_id_ = -1;
    max_link_id_ = -1;
    max_srlg_id_ = -1;
    std::vector<Link *> index;
    index.reserve(5000);
    for (Link &link : links_)
    {
        if (link.source_id > max_node_id_)
            max_node_id_ = link.source_id;
        if (link.ter_id > max_node_id_)
            max_node_id_ = link.ter_id;
        if (link.link_id > max_link_id_)
        {
            max_link_id_ = link.link_id;
        }
        if (link.srlg_num != 0)
        {
            for (int i = 0; i < link.srlgs.size(); ++i)
            {
                if (link.srlgs[i] > max_srlg_id_)
                {
                    max_srlg_id_ = link.srlgs[i];
                }
                srlg_group_[link.srlgs[i]].push_back(&link);
            }
        }

        nodes_.insert(link.source_id);
        nodes_.insert(link.ter_id);
    }
    // 为每个link设置一个专属的srlg，该srlg仅包含一条link.
    for (Link &link : links_)
    {
        link.srlg_num += 1;
        link.srlgs.push_back(++max_srlg_id_);
        srlg_group_[max_srlg_id_].push_back(&link);
    }
    size_ = max_node_id_ + 1;
    node_to_egress_links_.clear();
    node_to_ingress_links_.clear();
    node_to_egress_links_cost_.clear();
    node_to_ingress_links_cost_.clear();
    bp_node_to_egress_links_.clear();
    node_to_egress_links_.reserve(size_);
    node_to_ingress_links_.reserve(size_);
    node_to_egress_links_cost_.reserve(size_);
    node_to_ingress_links_cost_.reserve(size_);
    bp_node_to_egress_links_.reserve(size_);
    for (NodeId i = 0; i < size_; ++i)
    {
        // assert(nodes_.find(i) != nodes_.end());
        node_to_egress_links_.push_back({});
        bp_node_to_egress_links_.push_back({});
        node_to_ingress_links_.push_back({});
    }
    for (Link &link : links_)
    {
        node_to_egress_links_[link.source_id].push_back(&link);
        node_to_ingress_links_[link.ter_id].push_back(&link);
    }
    node_to_egress_links_cost_ = node_to_egress_links_;
    node_to_ingress_links_cost_ = node_to_ingress_links_;
}

void Graph::unsort()
{
    for (NodeId i = 0; i < size_; ++i)
    {
        std::sort(node_to_egress_links_[i].begin(),
                  node_to_egress_links_[i].end(), Uncompare);
        std::sort(node_to_ingress_links_[i].begin(),
                  node_to_ingress_links_[i].end(), Uncompare);
    }
}

void Graph::SortLinks()
{
    for (NodeId i = 0; i < size_; ++i)
    {
        std::sort(node_to_egress_links_[i].begin(),
                  node_to_egress_links_[i].end(), Compare);
        std::sort(node_to_ingress_links_[i].begin(),
                  node_to_ingress_links_[i].end(), Compare);
    }
}

void Graph::SortLinksLayer()
{
    for(NodeId i = 0; i < size_; ++i)
    {
        std::sort(node_to_egress_links_[i].begin(),
                  node_to_egress_links_[i].end(), CompareLayer);
        std::sort(node_to_ingress_links_[i].begin(),
                  node_to_ingress_links_[i].end(), CompareLayer);
    }
}

void Graph::SortLinks_cost()
{
    for(NodeId i = 0; i < size_; ++i)
    {
        std::sort(node_to_egress_links_cost_[i].begin(),
                  node_to_egress_links_cost_[i].end(),Compare);
        std::sort(node_to_ingress_links_cost_[i].begin(),
                  node_to_ingress_links_cost_[i].end(),Compare);
    }
}

void Graph::BuildMulGraph(const Flow& flow)
{
    mul_links_.clear();
    node_in_connecting_edge.clear();
    mul_node_to_egress_links_.clear();
    mul_node_to_ingress_links_.clear();
    int node_size = max_node_id_ + 1;
    int link_size = max_link_id_ + 1;
    layer = flow.SF_number;
    mul_links_.reserve(1000000);
    for(int i = 0; i < flow.SF_number + 1; i++)
    {
        for(Link &link : links_){
            mul_links_.emplace_back(link.link_id + i * link_size,
                    link.source_id + i * node_size,
                    link.ter_id + i * node_size,
                    link.cost,
                    link.bandwidth,
                    link.delay,link.srlg_num);
        }
        if(i != flow.SF_number)
        {
            for(std::unordered_set<int>::const_iterator it = flow.Subset_sequence[i].begin();it != flow.Subset_sequence[i].end();it++)
            {
                mul_links_.emplace_back(link_size * (flow.SF_number + 1) + i,
                    *it+i*node_size,
                    *it+(i+1)*(node_size),
                    node_to_weight_[*it].cost,0,node_to_weight_[*it].delay,0);
                mul_links_.back().weight_mask = 2;
                mul_links_.back().layer = i;
                // mul_links_.emplace_back(link_size * (flow.SF_number + 1) + i,
                //     *it+i*node_size,
                //     *it+(i+1)*(node_size),
                //     0,0,0,0);
                node_in_connecting_edge.insert(*it + i*node_size);
                node_in_connecting_edge.insert(*it + (i+1)*node_size);
            }
        }
    }

    int mul_size_ = node_size * (flow.SF_number + 1);
    mul_node_to_egress_links_.reserve(mul_size_);
    mul_node_to_ingress_links_.reserve(mul_size_);
    for (NodeId i = 0; i < mul_size_; ++i)
    {
        // assert(nodes_.find(i) != nodes_.end());
        mul_node_to_egress_links_.push_back({});
        mul_node_to_ingress_links_.push_back({});
    }
    for (Link &link : mul_links_)
    {
        mul_node_to_egress_links_[link.source_id].push_back(&link);
        mul_node_to_ingress_links_[link.ter_id].push_back(&link);
    }
    return;
}

void Graph::BuildMulGraphWithoutNodeWeight(const Flow& flow)
{
    mul_links_.clear();
    node_in_connecting_edge.clear();
    mul_node_to_egress_links_.clear();
    mul_node_to_ingress_links_.clear();
    int node_size = max_node_id_ + 1;
    int link_size = max_link_id_ + 1;
    layer = flow.SF_number;
    mul_links_.reserve(1000000);
    for(int i = 0; i < flow.SF_number + 1; i++)
    {
        for(Link &link : links_){
            mul_links_.emplace_back(link.link_id + i * link_size,
                    link.source_id + i * node_size,
                    link.ter_id + i * node_size,
                    link.cost,
                    link.bandwidth,
                    link.delay,link.srlg_num);
        }
        if(i != flow.SF_number)
        {
            for(std::unordered_set<int>::const_iterator it = flow.Subset_sequence[i].begin();it != flow.Subset_sequence[i].end();it++)
            {
                // mul_links_.emplace_back(link_size * (flow.SF_number + 1) + i,
                //     *it+i*node_size,
                //     *it+(i+1)*(node_size),
                //     node_to_weight_[*it].cost,0,node_to_weight_[*it].delay,0);
                mul_links_.emplace_back(link_size * (flow.SF_number + 1) + i,
                    *it+i*node_size,
                    *it+(i+1)*(node_size),
                    0,0,0,0);
                node_in_connecting_edge.insert(*it + i*node_size);
                node_in_connecting_edge.insert(*it + (i+1)*node_size);
            }
        }
    }

    int mul_size_ = node_size * (flow.SF_number + 1);
    mul_node_to_egress_links_.reserve(mul_size_);
    mul_node_to_ingress_links_.reserve(mul_size_);
    for (NodeId i = 0; i < mul_size_; ++i)
    {
        // assert(nodes_.find(i) != nodes_.end());
        mul_node_to_egress_links_.push_back({});
        mul_node_to_ingress_links_.push_back({});
    }
    for (Link &link : mul_links_)
    {
        mul_node_to_egress_links_[link.source_id].push_back(&link);
        mul_node_to_ingress_links_[link.ter_id].push_back(&link);
    }
    return;
}

void Graph::InitNetworkState(const Flow &flow)
{
    for (Link &link: links_)
    {
        link.cost = flow.bandwidth * 1.5 * std::pow(edge_cost_scale_factor, link.utilrate);
        link.delay = 1.5 * (1 / (1 - link.utilrate)) + link.const_delay;
    }
    for (NodeId i = 0; i < size_; ++i)
    {
        node_to_weight_[i].cost = flow.bandwidth * 5 * std::pow(node_cost_scale_factor, node_to_weight_[i].utilrate);
        node_to_weight_[i].delay = node_to_weight_[i].const_delay * (1 / (1 - node_to_weight_[i].utilrate));
    }
}

void Graph::ReleaseOldFlow()
{
    if (recent_flows.empty())
    {
        throw std::runtime_error("No flow to release.");
    }
    std::pair<Path, Flow> path_flow_pair = recent_flows.front();
    recent_flows.pop();
    for (Link *link : path_flow_pair.first.path_link)
    {
        link->utilrate -= path_flow_pair.second.bandwidth / link->bandwidth;
    }
    for (const NodeInfo &nodeinfo : path_flow_pair.first.path_used_function_nodes)
    {
        node_to_weight_[nodeinfo.node_id].utilrate -= 5 * path_flow_pair.second.bandwidth / node_to_weight_[nodeinfo.node_id].Capacity;
    }
}

void Graph::UpdateNetworkState(const Flow flow, const Path path)
{
    if(recent_flows.size() >= 3)
    {
        ReleaseOldFlow();
    }
    for(Link *link : path.path_link)
    {
        link->utilrate += flow.bandwidth / link->bandwidth;
        if (link->utilrate > 1.0) {
            std::cout<<flow.bandwidth << " " <<link->link_id<<" " <<link->bandwidth << " " << link->utilrate << std::endl;
            throw std::runtime_error("Utilization rate exceeds 100% on a link.");
        }
    }
    for(const NodeInfo &nodeinfo : path.path_used_function_nodes)
    {
        node_to_weight_[nodeinfo.node_id].utilrate += 5 * flow.bandwidth / node_to_weight_[nodeinfo.node_id].Capacity;
        if (node_to_weight_[nodeinfo.node_id].utilrate > 1.0) {
            throw std::runtime_error("Utilization rate exceeds 100% on a node.");
        }
    }
    Path path_copy = path;
    Flow flow_copy = flow;
    recent_flows.push({path_copy,flow_copy});
}

void Flow::Print() const
{
    std::cout << "\n"
              << "--------"
              << "\n";
    std::cout << "Flow Id: " << id << ", Source node: " << from
              << ", Destination node: " << to << ", Delay upperbound: "
              << delay_ub << ", Delay lowerbound: " << delay_lb
              << ", Bandwidth requirement: " << bandwidth << ",\n"
              << "Is Delaydiff: " << is_diff << ", Diff Range: "
              << diff << ", Separation type: " << type << ", Theory opt cost: "
              << opt_cost << ", Range Type: " << range_type << ", Min Cost Delay:"
              << min_cost_delay << ", Min Delay Delay: " << min_delay_delay << "\n";
}

bool Flow::CheckSfc(const std::vector<Link*> & cur_path) const
{
    int idx = 0;
    for(Link* link : cur_path)
    {
        if(Subset_sequence[idx].find(link->source_id) != Subset_sequence[idx].end())
        {
            idx++;
        }
        if(idx == SF_number){
            return true;
        }
    }
    return (Subset_sequence[idx].find(cur_path.back()->ter_id) != Subset_sequence[idx].end());
}

void Flow::PrintToCsv() const
{
    std::cout << id << "," << from << "," << to << "," << delay_ub
              << "," << delay_lb << "," << bandwidth << "," << is_diff << ","
              << diff << "," << type << "," << opt_cost << "," << range_type << ","
              << min_cost_delay << "," << min_delay_delay << ",";
}

Demand::Demand(const std::string &file_path)
{
    std::string line;
    std::ifstream in(file_path);
    if (in.fail())
    {
        std::cout << "File not found" << std::endl;
        return;
    }
    int case_cnt = 0;
    while (getline(in, line))
    {
        ++case_cnt;
        if (case_cnt == 1)
            continue;
        // 把line里的单元格数字字符提取出来，“,”为单元格分隔符
        std::vector<std::string> buffer;
        std::string str;
        for (int i = 0; i < line.size(); ++i)
        {
            if (line[i] == ',')
            {
                buffer.push_back(str);
                str.clear();
                continue;
            }
            str.push_back(line[i]);
        }
        buffer.push_back(str);
        assert(buffer.size() >= 6);
        flows_.emplace_back(
            atoi(buffer[0].c_str()), atoi(buffer[1].c_str()),
            atoi(buffer[2].c_str()), atof(buffer[3].c_str()),
            atof(buffer[4].c_str()), atoi(buffer[5].c_str()),
            atoi(buffer[6].c_str()), atof(buffer[7].c_str()),
            static_cast<DisjointType>(atoi(buffer[8].c_str())),
            atof(buffer[9].c_str()));

        flows_.back().is_SFC = false;
        if(buffer.size()>10){
            flows_.back().is_SFC = true;
            flows_.back().SF_number = atoi(buffer[10].c_str());
            flows_.back().Subset_sequence.reserve(flows_.back().SF_number);
            for(int i = 0; i < flows_.back().SF_number; i++)
            {
                flows_.back().Subset_sequence.push_back(std::unordered_set<int>());
            }
            std::string tmp;
            int cnt = 0;
            for (int i = 0; i < str.size(); ++i)
            {
                if ((str[i] == '|') || (str[i] == ','))
                {   flows_.back().Subset_sequence[cnt].insert(atoi(tmp.c_str()));
                    flows_.back().node_to_mask[atoi(tmp.c_str())] = 1<<cnt;
                    if(str[i] == '|')
                    {
                        cnt++;
                    }
                    tmp.clear();
                    continue;
                }
                tmp.push_back(str[i]);
            }
            flows_.back().Subset_sequence[cnt].insert(atoi(tmp.c_str()));
            flows_.back().node_to_mask[atoi(tmp.c_str())] = 1<<cnt;
            //std::cout<<cnt<<std::endl;
        }
    }

    in.close();
}

Demand::Demand(const std::string &file_path, Graph &graph)
{
    std::string line;
    std::ifstream in(file_path);
    if (in.fail())
    {
        std::cout << "File not found" << std::endl;
        return;
    }
    int case_cnt = 0;
    while (getline(in, line))
    {
        ++case_cnt;
        if (case_cnt == 1)
            continue;
        // 把line里的单元格数字字符提取出来，“,”为单元格分隔符
        std::vector<std::string> buffer;
        std::string str;
        for (int i = 0; i < line.size(); ++i)
        {
            if (line[i] == ',')
            {
                buffer.push_back(str);
                str.clear();
                continue;
            }
            str.push_back(line[i]);
        }
        buffer.push_back(str);
        assert(buffer.size() >= 6);
        // std::cout<<buffer[3].c_str()<<" "<<buffer[4].c_str()<<std::endl;
        flows_.emplace_back(
            atoi(buffer[0].c_str()), atoi(buffer[1].c_str()),
            atoi(buffer[2].c_str()), atof(buffer[3].c_str()),
            atof(buffer[4].c_str()), atoi(buffer[5].c_str()),
            atoi(buffer[6].c_str()), atof(buffer[7].c_str()),
            static_cast<DisjointType>(atoi(buffer[8].c_str())),
            atof(buffer[9].c_str()));

        flows_.back().is_SFC = false;
        if(buffer.size()>10){
            flows_.back().is_SFC = true;
            flows_.back().SF_number = atoi(buffer[10].c_str());
            flows_.back().Subset_sequence.reserve(flows_.back().SF_number);
            for(int i = 0; i < flows_.back().SF_number; i++)
            {
                flows_.back().Subset_sequence.push_back(std::unordered_set<int>());
            }
            // std::cout<<"size: "<<flows_.back().Subset_sequence.size()<<std::endl;
            std::string tmp;
            std::vector<int> sfc_requsets;
            sfc_requsets.reserve(flows_.back().SF_number);
            flows_.back().node_function_hash.clear();
            flows_.back().node_function_hash.resize(graph.GetMaxNodeId() + 1, std::vector<int>(flows_.back().SF_number, 0));
            // flows_.back().node_function_hash.reserve(graph.GetMaxNodeId()+1);
            // for(int i = 0; i < graph.GetMaxNodeId()+1; i++)
            // {
            //     flows_.back().node_function_hash.push_back({});
            //     flows_.back().node_function_hash[i].reserve(flows_.back().SF_number);
            //     for(int j = 0; j < flows_.back().SF_number; j++)
            //     {
            //         flows_.back().node_function_hash[i].push_back(0);
            //     }
            // }
            int cnt = 0;
            for (int i = 0; i < str.size(); ++i)
            {
                if ((str[i] == '|') || (str[i] == ','))
                {   
                    sfc_requsets.push_back(atoi(tmp.c_str()));
                    // flows_.back().Subset_sequence[cnt].insert(atoi(tmp.c_str()));
                    // flows_.back().node_to_mask[atoi(tmp.c_str())] = 1<<cnt;
                    if(str[i] == '|')
                    {
                        cnt++;
                    }
                    tmp.clear();
                    continue;
                }
                tmp.push_back(str[i]);
            }
            sfc_requsets.push_back(atoi(tmp.c_str()));
            for (int i = 0; i < sfc_requsets.size(); i++)
            {
                const std::vector<NodeId> node_to_function = graph.GetNodesforFunction(sfc_requsets[i]);
                for (NodeId node : node_to_function)
                {
                    flows_.back().Subset_sequence[i].insert(node);
                    // flows_.back().node_to_mask[node] = 1<<i;
                    flows_.back().node_function_hash[node][i] = 1;
                }
            }
            flows_.back().sfc_requsets = sfc_requsets;
            //std::cout<<cnt<<std::endl;
        }
    }
    
    in.close();
}

void ConflictStatus::Init(const Path &path)
{
    id_to_cnt_.clear();
    num_nozero_ids_ = 0;
    for (Link *link : path.path_link)
    {
        Add(link);
    }
}

void ConflictStatus::Init(const std::vector<Link *> &path)
{
    id_to_cnt_.clear();
    num_nozero_ids_ = 0;
    for (Link *link : path)
    {
        Add(link);
    }
}

void ConflictStatus::Add(const Link *link)
{
    for (int srlg_id : link->srlgs)
    {
        if (conflict_set_->CheckConflict(srlg_id))
        {
            if (id_to_cnt_[srlg_id] == 0)
            {
                ++num_nozero_ids_;
            }
            id_to_cnt_[srlg_id] += 1;
        }
    }
}
void ConflictStatus::Remove(const Link *link)
{
    for (int srlg_id : link->srlgs)
    {
        if (conflict_set_->CheckConflict(srlg_id))
        {
            id_to_cnt_[srlg_id] -= 1;
            if (id_to_cnt_[srlg_id] == 0)
            {
                --num_nozero_ids_;
            }
        }
    }
}

bool ConflictStatus::CoverConflictSet()
{
    return (num_nozero_ids_ >= conflict_set_->NumConflicts());
}

void Path::CompletePath()
{
    cost = 0;
    delay = 0;
    path_info.clear();
    path_info.push_back(path_link.front()->source_id);
    for (Link *link : path_link)
    {
        path_info.push_back(link->ter_id);
        delay += link->delay;
        cost += link->cost;
    }
}

void Path::CompletePathwithNodeWeight()
{
    cost = 0;
    delay = 0;
    path_info.clear();
    path_info.push_back(path_link.front()->source_id);
    for (Link *link : path_link)
    {
        path_info.push_back(link->ter_id);
        delay += link->delay;
        cost += link->cost;
    }
    for(const NodeInfo nodeinfo : path_used_function_nodes)
    {
        delay += nodeinfo.delay;
        cost += nodeinfo.cost;
    }
}

// bool VisitInfo::FastCheckDominanceAndUpdate(double delay, double cost) {
//     auto it = delay_to_cost_.lower_bound(delay);
//     if (it == delay_to_cost_.end() || it->first >= delay + range_) {
//         delay_to_cost_.emplace(delay, cost);
//         return true;
//     }
//     if (it->first == delay) {
//         return false;
//     }
//     auto it2 = delay_to_cost_.lower_bound(it->first - range_);
//     if (it2->first < delay) {
//         return false;
//     }
//     delay_to_cost_.emplace(delay, cost);
//     return true;
// }

bool VisitInfo::FastCheckDominanceAndUpdate(double delay, double cost)
{
    auto it = delay_to_cost_.lower_bound(delay);
    if (it == delay_to_cost_.end() || it->first >= delay + range_)
    {
        delay_to_cost_.emplace(delay, cost);
        return true;
    }
    if (it->first == delay)
    {
        return false;
    }
    if (it != delay_to_cost_.begin())
    {
        double range_lb = it->first - range_;
        --it;
        if (it->first >= range_lb)
        {
            return false;
        }
    }
    delay_to_cost_.emplace(delay, cost);
    return true;
}

bool VisitInfo::CheckDominanceAndUpdate(double delay, double cost)
{
    auto it = delay_to_cost_.lower_bound(delay);
    if (it == delay_to_cost_.end())
    {
        delay_to_cost_.emplace(delay, cost);
        return true;
    }
    if (it->first == delay)
    {
        if (it->second <= cost)
        {
            return false;
        }
        else
        {
            it->second = cost;
            return true;
        }
    }
    auto it2 = delay_to_cost_.upper_bound(delay - range_);
    while (it2->first < delay && it2->second > cost)
    {
        ++it2;
    }
    if (it2->first > delay)
    {
        delay_to_cost_.emplace(delay, cost);
        return true;
    }
    double range_ub = it2->first + range_;
    while (it->first <= range_ub && it->second > cost)
    {
        ++it;
    }
    if (it->first > range_ub)
    {
        delay_to_cost_.emplace(delay, cost);
        return true;
    }
    return false;
}

void VisitInfo::Process()
{
    auto it = delay_to_cost_.begin();
    delay_min_ = it->first - range_;
    min_cost_vector_.clear();
    min_cost_vector_.reserve(500);
    double delay = delay_min_;
    while (it != delay_to_cost_.end())
    {
        double min_cost = kMaxValue;
        auto it2 = it;
        // 加上1e-6解决浮点数的精度问题
        while (delay <= it->first + 1e-6)
        {
            while (it2 != delay_to_cost_.end() &&
                   it2->first < delay + range_ + delta_)
            {
                if (it2->second < min_cost)
                {
                    min_cost = it2->second;
                }
                ++it2;
            }
            min_cost_vector_.push_back(min_cost);
            delay += delta_;
        }
        ++it;
    }
}

double VisitInfo::GetMinCost(double delay_lb) const {
    if (delay_lb < delay_min_) {
        return kMaxValue;
    }
    int idx = DelayToIndex(delay_lb);
    if (idx >= min_cost_vector_.size()) {
        return kMaxValue;
    }
    return min_cost_vector_.at(idx);
}

bool VisitInfo::CheckMinCost(double delay_lb, double cost_ub) const
{
    if (delay_lb < delay_min_)
    {
        return false;
    }
    int idx = DelayToIndex(delay_lb);
    if (idx >= min_cost_vector_.size())
    {
        return false;
    }
    return min_cost_vector_.at(idx) < cost_ub;
}

void VisitInfo::Print() const
{
    for (auto &delay_and_cost : delay_to_cost_)
    {
        std::cout << "(" << delay_and_cost.first << ", "
                  << delay_and_cost.second << ") ";
    }
    std::cout << "\n";
}

void VisitInfo::SortEgressLinksBasedOnDelayBudget(
    const std::vector<Link*> all_egress_links,
    const std::vector<VisitInfo>& rechability_info) {
    delay_to_egress_links_.clear();
    delay_to_egress_links_.reserve(min_cost_vector_.size());
    double delay_lb = delay_min_;
    for (int j = 0; j < min_cost_vector_.size(); ++j) {
        delay_to_egress_links_.emplace_back();
        std::vector<Link*>& egress_links = delay_to_egress_links_.back();
        for (Link* link : all_egress_links) {
            NodeId u = link->ter_id;
            double cost = link->cost +
                rechability_info[u].GetMinCost(delay_lb - link->delay);
            link->weight = -cost;
        }
        egress_links = all_egress_links;
        std::sort(egress_links.begin(), egress_links.end(), Compare);
        delay_lb += delta_;
    }
}

const std::vector<Link *>& VisitInfo::GetEgressLinksBasedOnDelayLb(
    double delay_lb) const {
    if (delay_lb < delay_min_) {
        return empty_link_set_;
    }
    int idx = DelayToIndex(delay_lb);
    if (idx >= delay_to_egress_links_.size()) {
        return empty_link_set_;
    }
    return delay_to_egress_links_.at(idx);
}

bool QuickVisitInfo::FastCheckDominanceAndUpdate(double delay, double cost) {
    if (!delay_cost_pairs_.empty()) {
        if (delay_cost_pairs_.back().delay <= delay) {
            return false;
        }
        delay_gap_min_ = std::min(delay_min_ - delay, delay_gap_min_);
        delay_min_ = delay;
    } else {
        delay_gap_min_ = delay;
        delay_min_ = delay;
    }
    delay_cost_pairs_.emplace_back(delay, cost);
    return true;
}

void QuickVisitInfo::Process() {
    if (delay_gap_min_ <= 0) {
        return;
    }
    delta_ = delay_gap_min_ / kNumSlicesInRange;
    auto it = delay_cost_pairs_.begin();
    delay_max_ = it->delay;
    min_cost_vector_.clear();
    min_cost_vector_.reserve(100);
    double delay = delay_max_;
    // std::cout << delta_ << " and " << delay << "\n";
    while (it != delay_cost_pairs_.end()) {
        while (delay >= it->delay - 1e-6) {
            min_cost_vector_.push_back(it->cost);
            delay -= delta_;
        }
        ++it;
    }
}

double QuickVisitInfo::GetMinCost(double delay_ub) const {
    if (delay_ub >= delay_max_) {
        return min_cost_vector_.front();
    }
    int idx = DelayToIndex(delay_ub);
    if (idx >= min_cost_vector_.size()) {
        return kMaxValue;
    }
    return min_cost_vector_.at(idx);
}

bool QuickVisitInfo::CheckMinCost(double delay_ub, double cost_ub) const {
    if (min_cost_vector_.empty() || delay_ub < delay_min_) {
        return false;
    }
    if (delay_ub >= delay_max_) {
        return min_cost_vector_.front() < cost_ub;
    }
    int idx = DelayToIndex(delay_ub);
    // if (idx < 0 || idx >= min_cost_vector_.size()) {
    //     std::cout << idx << "\n";
    // }
    return min_cost_vector_.at(idx) < cost_ub;
}

void QuickVisitInfo::Print() const {
    for (auto &delay_and_cost : delay_cost_pairs_)
    {
        std::cout << "(" << delay_and_cost.delay << ", "
                << delay_and_cost.cost << ") ";
    }
    std::cout << "\n";
}
