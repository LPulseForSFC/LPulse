#ifndef SHORTEST_PATH_H_
#define SHORTEST_PATH_H_
#include <assert.h>
#include <functional>
#include <list>
#include <queue>
#include <unordered_set>
#include <vector>
#include "graph.h"

using CostFunc = std::function<double(Link*)>;

inline double LinkDelay(Link* link) {
    return link->delay;
}

inline double LinkCost(Link* link) {
    return link->cost;
}

class ShortestPath {
 public:
    virtual ~ShortestPath() {}
    virtual void InitWithDst(
        NodeId dst, const double* min_cost_to_dst = nullptr) = 0;
    // Return kMaxValue if there is no feasible path from @src.
    virtual double FindPathFromSrc(
        NodeId src, std::vector<Link*>* reverse_links) = 0;
};


class AStar : public ShortestPath {
 public:
    explicit AStar(Graph* graph, CostFunc f = LinkCost)
        : graph_(graph), cost_func_(f) {
        max_node_id_ = graph_->GetMaxNodeId();
        min_cost_to_dst_ = new double[max_node_id_ + 1];
    }
    ~AStar() {
        delete []min_cost_to_dst_;
    }
    void InitWithDst(
        NodeId dst, const double* min_cost_to_dst = nullptr) override;
    double FindPathFromSrc(
        NodeId src, std::vector<Link*>* reverse_links) override;

    const double* GetCostVector() {
        return min_cost_to_dst_;
    }

 private:
    Graph* graph_;
    NodeId max_node_id_;
    NodeId dst_;
    double* min_cost_to_dst_;
    CostFunc cost_func_;
};

class MulAStar : public ShortestPath {
 public:
    explicit MulAStar(Graph* graph, CostFunc f = LinkCost)
        : graph_(graph), cost_func_(f) {
        max_node_id_ = graph_->GetMaxNodeId() + graph_->GetlayerNumber() * graph_->NodeSize();
        min_cost_to_dst_ = new double[max_node_id_ + 1];
    }
    ~MulAStar() {
        delete []min_cost_to_dst_;
    }
    void InitWithDst(
        NodeId dst, const double* min_cost_to_dst = nullptr) override;
    double FindPathFromSrc(
        NodeId src, std::vector<Link*>* reverse_links) override
    {
        std::cout << "FindPathFromSrc is not implemented!\n";
        assert(false);
    }
    const double* GetCostVector() {
        return min_cost_to_dst_;
    }

 private:
    Graph* graph_;
    NodeId max_node_id_;
    NodeId dst_;
    double* min_cost_to_dst_;
    CostFunc cost_func_;
};

class Dijkstra : public ShortestPath {
 public:
    explicit Dijkstra(Graph* graph, CostFunc f = LinkCost)
        : graph_(graph), cost_func_(f) {
        max_node_id_ = graph_->GetMaxNodeId();
    }
    void InitWithDst(
        NodeId dst, const double* min_cost_to_dst = nullptr) override {
        dst_ = dst;
    }
    double FindPathFromSrc(
        NodeId src, std::vector<Link*>* reverse_links) override;

 private:
    Graph* graph_;
    NodeId max_node_id_;
    NodeId dst_;
    CostFunc cost_func_;
};

class BackwardDfs {
 public:
    BackwardDfs(Graph* graph, const Flow& flow)
        : graph_(graph), flow_(flow),
          visit_info_(graph->GetMaxNodeId() + 1,
                      VisitInfo(flow.delay_ub - flow.delay_lb)) {}

    const std::vector<VisitInfo>* GetReachabilityInfo();

 private:
    struct Snapshot {
        Link *link;
        int stage;
    };
    Graph* graph_;
    Flow flow_;
    std::vector<VisitInfo> visit_info_;
};

class ReachabilityInfo {
 public:
    ReachabilityInfo(Graph* graph, const Flow& flow)
        : graph_(graph), flow_(flow),
          visit_info_(graph->GetMaxNodeId() + 1) {}

    const std::vector<QuickVisitInfo>* GetReachabilityInfo();

 private:
    Graph* graph_;
    Flow flow_;
    std::vector<QuickVisitInfo> visit_info_;
};

class KShortestPath {
 public:
    explicit KShortestPath(Graph* graph, CostFunc f = LinkCost)
        : graph_(graph), cost_func_(f), astar_(graph, f),
          link_status_backup_(graph->GetMaxLinkId() + 1) {
        conflict_sets_.reserve(20);
        for (Link& link : graph_->GetMutableLinks()) {
            link_status_backup_[link.link_id] = link.status;
        }
    }
    ~KShortestPath() {
        if (astar2_) {
            delete astar2_;
        }
    }
    // Init computes the shortest path, and returns the cost
    double Init(NodeId src, NodeId dst);
    double FindNextPath();
    void AddSecondCostFunction(CostFunc g, double ub) {
        cost2_ub_ = ub;
        cost_func2_ = g;
        astar2_ = new AStar(graph_, g);
        astar2_->InitWithDst(dst_);
    }
    void AddConflictSet(const ConflictSet& conflict_set);
    const std::vector<Link*>& GetPath() {
        while (!path_heap_.top().path_ptr->valid) {
            path_heap_.pop();
        }
        return path_heap_.top().path_ptr->links;
    }

 private:
    struct ConditionalPath {
        explicit ConditionalPath(
            const std::vector<Link*>& reverse_links,
            const std::vector<Link*>& include = std::vector<Link*>(),
            const std::vector<Link*>& exclude = std::vector<Link*>())
            : included_links(include), excluded_links(exclude), valid(true) {
            links.reserve(include.size() + reverse_links.size());
            links = include;
            links.insert(links.end(), reverse_links.rbegin(),
                         reverse_links.rend());
        }
        std::vector<Link*> included_links;
        std::vector<Link*> excluded_links;
        std::vector<Link*> links;
        bool valid;
    };
    struct PathCostPair {
        friend bool operator<(const PathCostPair &a, const PathCostPair &b) {
            return a.cost > b.cost;
        }
        ConditionalPath* path_ptr;
        double cost;
    };
    std::vector<ConflictSet> conflict_sets_;
    std::list<ConditionalPath> all_paths_;
    std::priority_queue<PathCostPair> path_heap_;
    Graph* graph_;
    std::vector<LinkStatus> link_status_backup_;
    CostFunc cost_func_;
    AStar astar_;
    NodeId src_;
    NodeId dst_;

    CostFunc cost_func2_ = nullptr;
    double cost2_ub_ = kMaxValue;
    AStar* astar2_ = nullptr;
};

// Helper functions.
double ComputeCost(const std::vector<Link*>& links);
double ComputeDelay(const std::vector<Link*>& links);
// Calculate a good multiplier for lagrangian approach.
double CalculateMultiplier(Graph* graph, const Flow &flow);
// Helper functions for SFC.
void GetOriginalPath(Graph* graph, const std::vector<Link *> &path, const Flow &flow, std::vector<Link*> &original_path);
void GetUsedNodes(Graph* graph, const std::vector<Link*> &path, const Flow &flow, std::vector<NodeInfo> &used_nodes);
#endif  // SHORTEST_PATH_H_
