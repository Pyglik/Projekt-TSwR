import networkx as nx
import matplotlib.pyplot as plt

edges = [('m0', 'm1'), ('m1', 'm0'), ('m1', 'm2'), ('m1', 'm3'), ('m1', 'm4'), ('m2', 'm1'), ('m3', 'm1'), ('m4', 'm1'),
         ('s0', 's1'), ('s0', 's2'), ('s0', 's3'), ('s1', 's4'), ('s2', 's4'), ('s3', 's4')]

pos = {'m0': [-1, 1], 'm1': [-1, 0], 'm2': [-1.5, -1], 'm3': [-0.5, -1], 'm4': [-2, 0],
       's0': [1, 1], 's1': [0, 0], 's2': [1, 0], 's3': [2, 0], 's4': [1, -1]}

states = ['m0', 'm1', 'm2', 'm3', 'm4', 's0', 's1', 's2', 's3', 's4']


def draw_graph(state):
    node_color = ['#1f78b4']*10  # kolor domyślny
    if state:
        id = states.index(state)
        node_color[id] = 'r'  # zaznaczenie aktualnego stanu

    G = nx.DiGraph(edges)
    nx.draw(G, pos, node_color=node_color, with_labels=True)
    # plt.savefig('networkx.png')
    plt.ion()
    plt.show()
    plt.pause(0.001)
