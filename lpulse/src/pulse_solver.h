#ifndef PULSE_SOLVER_H_
#define PULSE_SOLVER_H_

#include "algorithm.h"
#include "graph.h"

#include <unordered_set>
#include <vector>

class Pulse : public Algorithm {
 public:
    Pulse() {}
    ~Pulse() {}
    void SetupTopology(Graph *graph) override {
        graph_ = graph;
    }
    Path FindPath(const Flow &flow) override;
};

class MulGraphPulse : public Pulse{
 public:
    Path FindPath(const Flow &flow) override;
 private:
    Graph *mul_graph_;
};
#endif  // PULSE_SOLVER_H_
