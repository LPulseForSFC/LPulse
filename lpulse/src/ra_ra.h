#ifndef RA_RA_H_
#define RA_RA_H_

#include <functional>
#include "algorithm.h"
#include "graph.h"
#include "shortest_path.h"

class MSG: public Algorithm{
 public:
    MSG(){}
    ~MSG() {}
    void SetupTopology(Graph *graph){
        graph_ = graph;
    }
    virtual Path FindPath(const Flow &flow) = 0;
 protected:
    Graph *graph_;
    Graph *LFG_;
    Path *ap_path_;
    Path *bp_path_;
    Flow LFG_flow_;
    CostFunc cost_func_;
    void LFG(const Flow &flow, CostFunc f = LinkCost);
    double ComputeBandwidth(const std::vector<Link*>& links);
    const std::vector<Link*> TransformPath(const std::vector<Link*>& links);
    std::vector<NodeInfo> GetUsedNodes(const std::vector<Link*>& links);
    std::unordered_map<NodeId, NodeId> node_lfg_to_g;
    std::vector<std::unordered_map<NodeId, NodeId>> node_g_to_lfg;
    std::vector<std::vector<Link*>> link_to_path_;
};

class RA: public MSG{
 public:
    Path FindPath(const Flow &flow) override;
};

class DelayRA: public MSG{
 public:
    Path FindPath(const Flow &flow) override;
};

#endif