import networkx as nx
import matplotlib.pyplot as plt

edges = [('m0', 'm1'), ('m1', 'm0'), ('m1', 'm2'), ('m1', 'm3'), ('m1', 'm4'), ('m2', 'm1'), ('m3', 'm1'), ('m4', 'm1'),
         ('s0', 's1'), ('s0', 's2'), ('s0', 's3'), ('s1', 's4'), ('s2', 's4'), ('s3', 's4')]

pos = {'m0': [-1, 1], 'm1': [-1, 0], 'm2': [-1.5, -1], 'm3': [-0.5, -1], 'm4': [-2.5, 0],
       's0': [1, 1], 's1': [0, 0], 's2': [1, 0], 's3': [2, 0], 's4': [1, -1]}

states = ['m0', 'm1', 'm2', 'm3', 'm4', 's0', 's1', 's2', 's3', 's4']

edge_names = {
    ('m0', 'm1'): 'Start / Stop',
    ('m1', 'm2'): 'Skręć w lewo /\n Koniec skr.',
    ('m1', 'm3'): 'Skręć w prawo /\n Koniec skr.',
    ('m1', 'm4'): 'Przeszkoda /\n Koniec omij.',
    ('s0', 's1'): 'Przeszkoda z prawej',
    ('s0', 's2'): 'Prz. z lew. lub z prz.',
    ('s0', 's3'): 'Przeszkoda z obu stron',
    ('s1', 's4'): 'Koniec skręcania',
    ('s2', 's4'): 'Koniec skręcania',
    ('s3', 's4'): 'Koniec zawracania'
}

state_names = {
    'm0': 'Robot bezczynny',
    'm1': 'Do przodu',
    'm2': 'Skr. w lewo',
    'm3': 'Skr. w prawo',
    'm4': 'Omij. przeszk.',
    's0': 'Omijanie przeszkody',
    's1': 'Skr. w lewo',
    's2': 'Skr. w prawo',
    's3': 'Zawracanie',
    's4': 'Koniec omijania'
}


def draw_graph(state):
    node_color = ['#1f78b4']*10  # kolor domyślny
    id = states.index('m0')
    node_color[id] = 'g'  # zaznaczenie stanu początkowego mastera
    id = states.index('s0')
    node_color[id] = 'g'  # zaznaczenie stanu początkowego slave'a
    if state:
        id = states.index(state)
        node_color[id] = 'r'  # zaznaczenie aktualnego stanu

    G = nx.DiGraph(edges)
    nx.draw(G, pos, node_size=1000, node_color=node_color, with_labels=False)
    nx.draw_networkx_labels(G, pos, state_names, font_size=10)
    nx.draw_networkx_edge_labels(G, pos, edge_names, font_size=8)
    plt.savefig('networkx.png')
    plt.ion()
    plt.show()
    plt.pause(0.001)
