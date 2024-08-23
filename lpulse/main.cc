#include <iostream>
#include <fstream>
#include <unordered_set>
#include <sstream>
#include "src/ap_path.h"
#include "src/bp_path.h"
#include "src/graph.h"
#include "src/ksp_solver.h"
#include "src/shortest_path.h"
#include "src/pulse_solver.h"
#include "src/ra_ra.h"

void test(std::string topo_path, std::string node_path, std::string tunnel_path, int type_id, int flow_id = 0)
{
    Graph graph(topo_path, node_path);
    // std::cout << "\n--------------------------\n";
    // std::cout << topo_path << std::endl;
    // std::cout << "Number of Nodes: " << graph.NodeSize()
    //           << "\nNumber of links: " << graph.LinkSize() << std::endl;
    Demand demand(tunnel_path, graph);
    // std::cout << "Number of flows: " << demand.NumFlows() << std::endl;
    // if (flow_id < 0 || flow_id >= demand.NumFlows()){
    //     std::cout<<"Invalid flow id"<<std::endl;
    //     return;
    // }
    std::vector<int> flow_ids;
    if (flow_id == 0) {
        flow_ids.reserve(demand.NumFlows());
        for (int i = 0; i < demand.NumFlows(); ++i) {
            flow_ids.push_back(i);
        }
    } else {
        flow_ids.push_back(flow_id - 1);
    }
    for (int i : flow_ids){
        // std::cout<<demand.GetFlow(i).delay_ub<<std::endl;
        switch (type_id)
        {
            case 1:{
                SfcKsp pulse;
                pulse.SetupTopology(&graph);
                Path path = pulse.FindPath(demand.GetFlow(i));
                pulse.PrintToCsv();
                path.Print();
                break;
            }
            case 2:{
                SfcLagrangianKsp pulse;
                pulse.SetupTopology(&graph);
                Path path = pulse.FindPath(demand.GetFlow(i));
                pulse.PrintToCsv();
                path.Print();
                break;
            }
            case 3:{
                MulGraphPulse pulse;
                pulse.SetupTopology(&graph);
                Path path = pulse.FindPath(demand.GetFlow(i));
                pulse.PrintToCsv();
                path.Print();
                break;
            }
            case 4:{
                RA ra;
                ra.SetupTopology(&graph);
                Path path = ra.FindPath(demand.GetFlow(i));
                ra.PrintToCsv();
                path.Print();
                break;
            }
            case 5:{
                DelayRA ra;
                ra.SetupTopology(&graph);
                Path path = ra.FindPath(demand.GetFlow(i));
                ra.PrintToCsv();
                path.Print();
                break;
            }

        }
    }

}

int main(int argc, char *argv[])
{
    int type_id = atoi(argv[4]);
    int flow_id = atoi(argv[5]);
    clock_t start_time = clock();
    test(argv[1], argv[2],argv[3], type_id, flow_id);
    clock_t end_time = clock();
    return 0;
}
