#ifndef BP_PATH_H_
#define BP_PATH_H_
#include <assert.h>
#include <functional>
#include <vector>
#include "graph.h"
#include "shortest_path.h"

class BpPath {
 public:
    virtual ~BpPath() {}
    virtual double FindBpPath(
        const std::vector<Link*>& ap_path, const Flow& flow, int &iteration_num) = 0;
    virtual ConflictSet GetConflictSet() = 0;
    virtual std::vector<Link*> GetBpPath() = 0;
};

// DisjointBp guarantees optimality, but may take longer time.
class DisjointBp : public BpPath {
 public:
    explicit DisjointBp(Graph* graph)
        : graph_(graph) {}
    ~DisjointBp() {}
    double FindBpPath(
        const std::vector<Link*>& ap_path, const Flow& flow, int &iteration_num) override;
    ConflictSet GetConflictSet() override {
        std::cout << "DisjointBp does not compute conflict set.\n";
        assert(false);
    }
    std::vector<Link*> GetBpPath() override {
        return path_;
    }

 private:
    struct Snapshot {
        Link *link;
        int stage;
    };
    Graph* graph_;
    std::vector<Link*> path_;
};

// SrlgDisjointBp returns a path for each feasible case,
// and returns a conflict set for each infeasible case.
// SrlgDisjointBp can be also used for finding link disjoint bp path.
// However, the efficiency is much lower.
class SrlgDisjointBp : public BpPath {
 public:
    SrlgDisjointBp(
        Graph* graph, const double* min_delay_to_dst = nullptr)
        : graph_(graph), astar_(graph, LinkDelay),
          base_min_delay_to_dst_(min_delay_to_dst),
          conflict_set_(graph_->GetMaxSrlgId() + 1) {
    }
    ~SrlgDisjointBp() {}
    double FindBpPath(
        const std::vector<Link*>& ap_path, const Flow& flow, int &iteration_num) override;
    ConflictSet GetConflictSet() override {
        return conflict_set_;
    }
    std::vector<Link*> GetBpPath() override {
        return path_;
    }

 private:
    struct Snapshot {
        Link *link;
        int stage;
    };
    Graph* graph_;
    std::vector<Link*> path_;
    AStar astar_;
    ConflictSet conflict_set_;
    const double* base_min_delay_to_dst_;
    std::vector<std::vector<bool>> visit_cnt_;
};

class KspBp : public BpPath {
 public:
    explicit KspBp(Graph* graph) : graph_(graph) {}
    ~KspBp() {}
    double FindBpPath(
        const std::vector<Link*>& ap_path, const Flow& flow, int &iteration_num) override;
    ConflictSet GetConflictSet() override {
        std::cout << "KspBp does not compute conflict set.\n";
        assert(false);
    }
    std::vector<Link*> GetBpPath() override {
        return path_;
    }

 private:
    Graph* graph_;
    std::vector<Link*> path_;
};

#endif  // BP_PATH_H_
