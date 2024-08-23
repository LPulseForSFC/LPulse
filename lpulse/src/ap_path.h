#ifndef AP_PATH_H_
#define AP_PATH_H_
#include <assert.h>
#include <functional>
#include <unordered_set>
#include <vector>
#include "bp_path.h"
#include "graph.h"
#include "shortest_path.h"

class ApPath
{
public:
    virtual ~ApPath() {}
    // If bp_path_solver == nullptr, then ApPath only finds one path.
    virtual void Init(Graph *graph, BpPath *bp_path_solver,
                      const double *min_cost_to_dst,
                      const double *min_delay_to_dst);
    // Find an optimal path for @flow with cost smaller than @cost_ub.
    virtual double FindOptPath(
        const Flow &flow, double cost_ub, LogInfo &ap_info, LogInfo &bp_info) = 0;
    // Find a close-to-optimal path for @flow with cost smaller
    // than @cost_ub.
    virtual double FindCloseToOptPath(
        const Flow &flow, double cost_ub, LogInfo &ap_info, LogInfo &bp_info) = 0;
    virtual std::vector<Link *> GetApPath()
    {
        return ap_path_;
    }
    virtual std::vector<Link *> GetBpPath()
    {
        assert(bp_path_solver_ != nullptr);
        return bp_path_;
    }

protected:
    struct Snapshot
    {
        Link *link;
        int stage;
    };
    Graph *graph_;
    BpPath *bp_path_solver_;
    const double *min_cost_to_dst_;
    const double *min_delay_to_dst_;
    std::vector<Link *> ap_path_;
    std::vector<Link *> bp_path_;
};

// GenericAp guarantees optimality, but may take longer time.
class GenericAp : public ApPath
{
public:
    GenericAp() {}
    double FindOptPath(
        const Flow &flow, double cost_ub, LogInfo &ap_info, LogInfo &bp_info) override;
    double FindCloseToOptPath(
        const Flow &flow, double cost_ub, LogInfo &ap_info, LogInfo &bp_info) override;

private:
    // The following variable is only used in FindCloseToOptPath.
    std::vector<std::vector<double>> visited_best_cost_;
};

class MaskAp
{
    public:
    MaskAp(){}
    ~MaskAp(){}
    void Init(Graph *graph, BpPath *bp_path_solver,const Flow &flow,
                    const std::vector<std::unordered_map<NodeId,const double *>> node_to_min_cost_to_dst,
                    const std::vector<std::unordered_map<NodeId,const double *>> node_to_min_delay_to_dst);
    double FindOptPath(
        const Flow &flow, double cost_ub, LogInfo &ap_info, LogInfo &bp_info);
    std::vector<Link *>& GetApPath()
    {
        return ap_path_;
    }
    std::vector<NodeInfo>& GetUsedNodes()
    {
        return used_nodes_;
    }

private:
    void GetDelayCostArray(const Flow &flow);
    Graph *graph_;
    BpPath *bp_path_solver_;
    std::vector<std::unordered_map<NodeId,const double *>> node_to_min_cost_to_dst_;
    std::vector<std::unordered_map<NodeId,const double *>> node_to_min_delay_to_dst_;
    std::vector<int> DelayArray;
    std::vector<int> CostArray;
    std::vector<NodeInfo> used_nodes_;
    std::vector<Link *> ap_path_;
    std::vector<Link *> bp_path_;
    struct FlagSnapshot
    {
        Link *link;
        int stage;
        bool flag;
        int step;
    };
    std::vector<std::vector<std::vector<int>>> visited_best_cost_;
};

class MulGraphAp : public GenericAp
{
 public:
    MulGraphAp() {}
    void Init(Graph *graph, BpPath *bp_path_solver, const Flow &flow,
                  const double *min_cost_to_dst,
                  const double *min_delay_to_dst);
    double FindOptPath(
        const Flow &flow, double cost_ub, LogInfo &ap_info, LogInfo &bp_info) override;
 private:
    void InterEdgeOnly(Graph *graph, const Flow &flow);
    std::vector<std::vector<double>> visited_best_cost_;
};

class MulGraphMaskAp : public GenericAp
{
 public:
    MulGraphMaskAp() {}
    void Init(Graph *graph, BpPath *bp_path_solver, const Flow &flow,
                const double *min_cost_to_dst,
                const double *min_delay_to_dst);
    double FindOptPath(
        const Flow &flow, double cost_ub, LogInfo &ap_info, LogInfo &bp_info) override;

    std::vector<NodeInfo>& GetUsedNodes()
    {
        return used_nodes_;
    }
 private:
    std::vector<std::vector<std::vector<int>>> visited_best_cost_;
    std::vector<std::vector<double>> step_cost_;
    std::vector<std::vector<double>> step_delay_;
    std::vector<NodeInfo> used_nodes_;
    struct FlagSnapshot
    {
        Link *link;
        int stage;
        bool flag;
        int step;
    };
};
#endif // AP_PATH_H_
