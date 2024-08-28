# LPulse
This is the code repository for the paper "LPulse: An Efficient Algorithm for Service Function Chain Placement and Routing with Delay Guarantee." LPulse is an algorithm for solving the Delay-Constrained Service Function Chain Placement and Routing (DC-SFCPR) problem. LPulse efficiently solves the DC-SFCPR problem with optimality guarantees and demonstrates good scalability to large topologies. Our LPulse repository is developed based on [Pulse+](https://github.com/nothepeople/drcr.git).

## code structure
```
.
├── Data                # Data used for testing
│   ├── Random
│   ├── Real
│   └── Real_strict
├── Optimize            # Baseline methods: Optimization method & Randomized rounding
│   ├── main.py
│   ├── routing.py
│   └── utils.py
├── README.md
└── lpulse              # LPulse algorithm, along with baseline methods including Ksp, LagrangianKsp, MSG, and D-MSG
    ├── main.cc
    ├── makefile
    └── src
```

## Software requirements
1. **C++ Compiler (GCC - GNU Compiler Collection)**: On Linux systems, you can install it using the following commands:
   ```bash
   sudo apt-get update
   sudo apt-get install g++
   ```

2. **Make Tool**: On Linux systems, you can install it using the following command:
   ```bash
   sudo apt-get install make
   ```
3. (Only for baseline methods: Optimization method & Randomized rounding) Install [Gurobi Solver](https://www.gurobi.com). You can request a free academic license [here](https://www.gurobi.com/academia/academic-program-and-licenses/).

## Try our Code
#### Evaluating LPulse/Ksp/LagrangianKsp/MSG/D-MSG
```
cd lpulse
make main
./main "TopoFilePath" "NodeFilePath" "TunnelFilePath" "MethodID" "FlowID"
```
In main.cc, you can try different methods using different MethodIDs.  You can choose 1 for Ksp, 2 for LagtangianKsp, 3 for LPulse, 4 for MSG, 5 for D-MSG.


For example, to evaluate LPulse for the first case in the directory Data/Random/node1000_ER_Case0, you could run:
```
$ ./main ../Data/Random/node1000_ER_Case0/topo.csv ../Data/Random/node1000_ER_Case0/node_capacity.csv ../Data/Random/node1000_ER_Case0/tunnel.csv 3 1
```
You will obtain the algorithm's iteration count, solution time (in microseconds), path information, total cost, and total delay, as shown below:
```
2518,19621,0,0
Path:
Node List: 26->98->774->98->26->391->588->629->257
Link List: 346->1326->1327->347->354->4344->5678->3135
Function Node List: 774|98|391.
Cost is: 562, Delay is: 255.
```
#### Evaluating Optimization method/Randomized rounding
```
cd Optimize
python main.py "TopoDirPath" "MethodID" "FlowID"
```
In main.cc, you can try different methods using different MethodIDs.  You can choose 1 for Optimization method, 2 for Randomized rounding.
For example, to evaluate Optimization method for the first case in the directory Data/Random/node1000_ER_Case0, you could run:
```
$ python main.py ../Data/Random/node1000_ER_Case0 1 1
```
You will obtain the algorithm's solution time (in microseconds).

## Citation
If you use our code, please cite our paper:
```
@article{liu2024lpulse,
  title={LPulse: An efficient algorithm for service function chain placement and routing with delay guarantee},
  author={Liu, Ximeng and Zhao, Shizhen and Wang, Xinbing and Zhou, Chenghu},
  journal={Computer Networks},
  pages={110728},
  year={2024},
  publisher={Elsevier}
}
```