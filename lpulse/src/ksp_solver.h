#ifndef KSP_SOLVER_H_
#define KSP_SOLVER_H_

#include "algorithm.h"
#include "graph.h"

class DelayKsp : public Algorithm
{
public:
    DelayKsp() {}
    ~DelayKsp() {}
    void SetupTopology(Graph *graph) override
    {
        graph_ = graph;
    }
    PathPair FindPathPair(const Flow &flow) override;
    Path FindPath(const Flow &flow) override;

private:
    Graph *graph_;
    Path ap_path_;
    Path bp_path_;
};

class CostKsp : public Algorithm
{
public:
    CostKsp() {}
    ~CostKsp() {}
    void SetupTopology(Graph *graph) override
    {
        graph_ = graph;
        ap_path_ = nullptr;
        bp_path_ = nullptr;
    }
    PathPair FindPathPair(const Flow &flow) override;
    Path FindPath(const Flow &flow) override;

protected:
    Graph *graph_;
    Path *ap_path_;
    Path *bp_path_;
};

class CostKspPulse : public CostKsp {
 public:
    Path FindPath(const Flow &flow) override;
    PathPair FindPathPair(const Flow &flow) override;
};

class LagrangianKsp : public Algorithm
{
public:
    explicit LagrangianKsp(double u = kMaxValue) : default_u_(u) {}
    ~LagrangianKsp() {}
    void SetupTopology(Graph *graph) override
    {
        graph_ = graph;
    }
    PathPair FindPathPair(const Flow &flow) override;
    Path FindPath(const Flow &flow) override;

protected:
    Graph *graph_;
    double default_u_;
    Path *ap_path_;
    Path *bp_path_;
};

class SfcKsp: public CostKsp
{
public:
    void SetupTopology(Graph *graph) override
    {
        graph_ = graph;
        ap_path_ = nullptr;
        bp_path_ = nullptr;
    }
    Path FindPath(const Flow &flow) override;
private:
    Graph *mul_graph_;
};

class SfcLagrangianKsp: public LagrangianKsp
{
public:
    Path FindPath(const Flow &flow) override;
private:
    Graph *mul_graph_;
};
#endif // KSP_SOLVER_H_
