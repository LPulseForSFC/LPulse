import pandas as pd
import os
import networkx as nx


def read_graph(topo_dir):
    topo_path = os.path.join(topo_dir, 'topo.csv')
    node_capacity_path = os.path.join(topo_dir, 'node_capacity.csv')
    topo_df = pd.read_csv(topo_path)
    node_capacity_df = pd.read_csv(node_capacity_path)
    topo_df['Bandwidth'] = topo_df['Bandwidth'].astype(float) / 1e9
    node_capacity_df['Capacity'] = node_capacity_df['Capacity'].astype(float) / 1e9
    G = nx.DiGraph()
    for idx, row in node_capacity_df.iterrows():
        G.add_node(int(row['NodeID']), Capacity=row['Capacity'], Delay=row['Delay'], Cost=row['Cost'], UtilRate = 0)
    for idx, row in topo_df.iterrows():
        G.add_edge(int(row['SourceID']), int(row['DestinationID']), Bandwidth=row['Bandwidth'], Delay=row['Delay'], \
                    Cost=row['Cost'], UtilRate = 0)
    return G

def read_request(topo_dir):
    request_path = os.path.join(topo_dir, 'tunnel.csv')
    request_df = pd.read_csv(request_path)
    request_df['Bandwidth'] = request_df['Bandwidth'].astype(float) / 1e9
    return request_df

def get_function_to_node(topo_dir, function_num=6):
    node_capacity_path = os.path.join(topo_dir, 'node_capacity.csv')
    node_capacity_df = pd.read_csv(node_capacity_path)
    node_capacity_df['Function'] = node_capacity_df['Function'].astype(str)
    function_to_node = {i: [] for i in range(function_num)}
    for idx, row in node_capacity_df.iterrows():
        node_to_function = [] if pd.isna(row['Function']) or row['Function'] == 'nan' \
            else [int(float(i)) for i in row['Function'].split('|')]
        for function_id in node_to_function:
            function_to_node[function_id].append(row['NodeID'])
    return function_to_node

def get_layer_graph(G, sfc_request, num_node, function_to_node, sfc_request_num=3):
    layer_graph = nx.DiGraph()
    origin_nodes = list(G.nodes)
    origin_edges = list(G.edges)
    for i in range(sfc_request_num + 1):
        nodes = [(i * num_node + n, {'Cost':G.nodes[n]['Cost'],'Delay':G.nodes[n]['Delay']}) for n in origin_nodes]
        edges = [(i * num_node + u, i * num_node + v,{'Cost':G.edges[(u,v)]['Cost'], 'Delay':G.edges[(u,v)]['Delay']}) for u, v in origin_edges]
        layer_graph.add_nodes_from(nodes)
        layer_graph.add_edges_from(edges)
    for i in range(sfc_request_num):
        for node_id in function_to_node[sfc_request[i]]:
            layer_graph.add_edge(i * num_node + node_id, (i + 1) * num_node + node_id,Cost=G.nodes[node_id]['Cost'],Delay=G.nodes[node_id]['Delay'])
    return layer_graph

def get_path_on_original_graph(num_node, path_on_layer_graph):
    path_on_layer_graph = [path_on_layer_graph[0][0]] + [path_on_layer_graph[i][1] for i in range(len(path_on_layer_graph))]
    raw_path = [node_id % num_node for node_id in path_on_layer_graph]

    used_function_nodes = []
    for i in range(len(raw_path) - 1):
        u, v = raw_path[i], raw_path[i + 1]
        if u == v:
            used_function_nodes.append(u)
    
    path = [raw_path[0]]
    pre_node = raw_path[0]
    for node in raw_path[1:]:
        if node != pre_node:
            path.append(node)
            pre_node = node
    return path, used_function_nodes

# def store_result(result_lists, store_path):
#     with open(store_path, 'w') as file:
#         for item in result_lists:
#             file.write(f"1,{item},0,0\n")