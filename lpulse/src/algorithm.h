#ifndef ALGORITHM_H_
#define ALGORITHM_H_
#include <assert.h>
#include <fstream>
#include <sstream>
#include "graph.h"

const int kMaxDelay = 1000;
const int kMaxNumConflictSet = 6;

class Algorithm
{
public:
   virtual void SetupTopology(Graph *graph) = 0;
   // Return an empty path if there is no feasible path for @flow.
   virtual Path FindPath(const Flow &flow)
   {
      std::cout << "FindPath Not Implemented!" << std::endl;
      assert(false);
   }
   virtual PathPair FindPathPair(const Flow &flow)
   {
      std::cout << "FindPathPair Not Implemented!" << std::endl;
      assert(false);
   }
   virtual void PrintLog()
   {
      if (ap_info_.iteration_num != 0)
      {
         std::cout << "Ap Path:"
                   << "iteration_num: " << ap_info_.iteration_num
                   << " Ap Time: " << ap_info_.total_time / CLOCKS_PER_SEC * 1000
                   << "(ms)." << std::endl;
      }
      if (bp_info_.iteration_num != 0)
      {
         std::cout << "Bp Path:"
                   << "iteration_num: " << bp_info_.iteration_num
                   << " Bp Time: " << bp_info_.total_time / CLOCKS_PER_SEC * 1000
                   << "(ms)." << std::endl;
      }
      std::fstream Outfile("cost_ksp", std::ofstream::app);
      Outfile << ap_info_.iteration_num << "," << ap_info_.total_time << ","
              << bp_info_.iteration_num << "," << bp_info_.total_time << std::endl;
   }
   virtual void PrintToCsv()
   {
      std::cout << ap_info_.iteration_num << "," << ap_info_.total_time << ","
              << bp_info_.iteration_num << "," << bp_info_.total_time << std::endl;
   }
   virtual clock_t GetApTime()
   {
      return ap_info_.total_time;
   }

   virtual void PrintStackDepth()
   {
      std::cout << ap_info_.max_stack_depth << std::endl;
   }
   virtual ~Algorithm() {}

protected:
   Graph *graph_;
   LogInfo ap_info_;
   LogInfo bp_info_;
   bool force_quit_;
};

#endif // ALGORITHM_H_
