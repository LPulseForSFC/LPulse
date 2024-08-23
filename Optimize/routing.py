import gurobipy as grb

def Routing(V,E,c_e,d_e,c_v,d_v,s,t,D,F):
    m = grb.Model('routing')
    x = m.addVars(E, vtype=grb.GRB.BINARY, name="x")
    m.setObjective(x.prod(c_e), grb.GRB.MINIMIZE)
    m.Params.OutputFlag = 0
    #Delay Constraint
    m.addConstr(x.prod(d_e)<=D, "DelayConstraint")

    for k in V:
        if k == s:
            m.addConstr(x.sum(s, '*')-x.sum('*', s) == 1, "FlowOutSource")
        elif k == t:
            m.addConstr(x.sum('*', t)-x.sum(t, '*') == 1, "FlowIntoSink")
        else:
            m.addConstr(x.sum('*',k) == x.sum(k,'*'),"FlowConservation_"+str(k))

    m.optimize()
    if m.status == grb.GRB.OPTIMAL:
        #print("Optimal solution found with total cost:", m.objVal)
        path = []
        current_node = s
        while current_node != t:
            for u, v in E:
                if x[u,v].X > 0.5 and u == current_node:
                    path.append((u,v))
                    current_node = v
                    break
        return path
    else:
        print("No feasible solution found")

def Randomized_Rounding_Routing(V,E,c_e,d_e,c_v,d_v,s,t,D,F):
    m = grb.Model('routing')
    x = m.addVars(E, vtype=grb.GRB.CONTINUOUS, lb=0, ub=1, name="x")
    m.setObjective(x.prod(c_e), grb.GRB.MINIMIZE)
    m.Params.OutputFlag = 0
    #Delay Constraint
    m.addConstr(x.prod(d_e)<=D, "DelayConstraint")

    for k in V:
        if k == s:
            m.addConstr(x.sum(s, '*')-x.sum('*', s) == 1, "FlowOutSource")
        elif k == t:
            m.addConstr(x.sum('*', t)-x.sum(t, '*') == 1, "FlowIntoSink")
        else:
            m.addConstr(x.sum('*',k) == x.sum(k,'*'),"FlowConservation_"+str(k))

    m.optimize()
    if m.status == grb.GRB.OPTIMAL:
        #print("Optimal solution found with total cost:", m.objVal)
        path = []
        current_node = s
        while current_node != t:
            for u, v in E:
                if x[u,v].X > 0.5 and u == current_node:
                    path.append((u,v))
                    current_node = v
                    break
            return []
        return path
    else:
        print("No feasible solution found")