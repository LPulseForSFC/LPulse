from routing import Routing, Randomized_Rounding_Routing
from utils import read_graph,read_request,get_function_to_node,get_layer_graph,get_path_on_original_graph
import sys
import os
import time

if __name__=="__main__":
    topo_path = sys.argv[1]
    method_id = sys.argv[2]
    tunnel_id = int(sys.argv[3])
    g = read_graph(topo_path)
    df = read_request(topo_path)
    function_to_node = get_function_to_node(topo_path)
    sfc_request_num = 3
    num_node = g.number_of_nodes()
    row = df.iloc[tunnel_id]
    # for idx, row in df.iterrows():
    start_time = time.time()
    SfcRequests = [int(i) for i  in row['SfcRequest'].split('|')]
    F = []
    for request in SfcRequests:
        F.append(function_to_node[request])
    G = get_layer_graph(g, SfcRequests, g.number_of_nodes(), function_to_node)
    V = list(G.nodes())
    E = list(G.edges())
    c_e = {(u,v):data['Cost'] for u,v,data in G.edges(data=True)}
    d_e = {(u,v):data['Delay'] for u,v,data in G.edges(data=True)}
    c_v = {(fn,n):G.nodes[n]['Cost'] for fn, nodes in enumerate(F) for n in nodes}
    d_v = {(fn,n):G.nodes[n]['Delay'] for fn, nodes in enumerate(F) for n in nodes}
    s = int(row['SourceID'])
    t = int(row['DestinationID']) + sfc_request_num * num_node
    D = float(row['MaxDelay'])
    if method_id == '0':
        path = Routing(V,E,c_e,d_e,c_v,d_v,s,t,D,F)
        original_path,_ = get_path_on_original_graph(num_node, path)
    elif method_id == '1':
        path = Randomized_Rounding_Routing(V,E,c_e,d_e,c_v,d_v,s,t,D,F)
    end_time = time.time()
    solving_time = int((end_time-start_time) * 10e6)
    print(f'1,{solving_time},0,0')
