from statemachine import State
from generator import Generator, create_transitions
from graph import draw_graph

# define states for a master (way of passing args to class)
master_options = [
    {'name': 'Robot bezczynny', 'initial': True},  # 0
    {'name': 'Jazda do przodu'},  # 1
    {'name': 'Skręcanie w lewo'},  # 2
    {'name': 'Skręcanie w prawo'},  # 3
    {'name': 'Omijanie przeszkody'}  # 4
]

# valid transitions for a master (indices of states from-to)
master_from_to = [
    [0, [1]],
    [1, [0, 2, 3, 4]],
    [2, [1]],
    [3, [1]],
    [4, [1]],
]

master_events = {
    'm_0_1': 'Start',
    'm_1_0': 'Stop',
    'm_1_2': 'Skręć w lewo',
    'm_1_3': 'Skręć w prawo',
    'm_1_4': 'Przed robotem wykryto przeszkodę',
    'm_2_1': 'Koniec skręcania',
    'm_3_1': 'Koniec skręcania',
    'm_4_1': 'Koniec omijania'
}

# create State objects for a master
# ** -> unpack dict to args
master_states = [State(**opt) for opt in master_options]
# create transitions for a master (as a dict)
master_transitions = create_transitions(master_states, master_from_to, master=True)

# define states for a slave (way of passing args to class)
slave_options = [
    {'name': 'Omijanie przeszkody', 'initial': True},  # 0
    {'name': 'Skręcanie w lewo'},  # 1
    {'name': 'Skręcanie w prawo'},  # 2
    {'name': 'Zawracanie'},  # 3
    {'name': 'Koniec omijania'}  # 4
]

# valid transitions for a master (indices of states from-to)
slave_from_to = [
    [0, [1, 2, 3]],
    [1, [4]],
    [2, [4]],
    [3, [4]],
    [4, []],
]

slave_events = {
    's_0_1': 'Przeszkoda wykryta z prawej',
    's_0_2': 'Przeszkoda wykryta z lewej lub z przodu',
    's_0_3': 'Przeszkoda wykryta z obu stron',
    's_1_4': 'Koniec skręcania',
    's_2_4': 'Koniec skręcania',
    's_3_4': 'Koniec zawracania'
}

# create State objects for a master
# ** -> unpack dict to args
slave_states = [State(**opt) for opt in slave_options]
# create transitions for a master (as a dict)
slave_transitions = create_transitions(slave_states, slave_from_to, master=False)

# create path from transitions (exemplary)
path = ['m_0_1', 'm_1_2', 'm_2_1', 'm_1_4', 's_0_2', 's_2_4', 'm_1_3', 'm_3_1', 'm_1_0']

master = Generator.create_master(master_states, master_transitions)
slave = Generator.create_master(slave_states, slave_transitions)

i = 0
master_on = True
while True:
    if master_on:
        state_machine = master
        events = master_events
    else:
        state_machine = slave
        events = slave_events

    print('Aktualny stan:', state_machine.current_state.name)
    tranzycje = state_machine.allowed_transitions

    id = state_machine.states.index(state_machine.current_state)
    if master_on:
        state = 'm'+str(id)
    else:
        state = 's'+str(id)
    draw_graph(state)

    # Ręczne wybieranie tranzycji
    print('Dostępne tranzycje:')
    for i in range(len(tranzycje)):
        print(str(i + 1) + '.', events[tranzycje[i].identifier], '->', tranzycje[i].destinations[0].name)

    print('Wybierz zdarzenie:', end=' ')
    zd = int(input()) - 1
    while zd not in range(len(tranzycje)):
        print('Niepoprawne zdarzenie.')
        print('Podaj numer zdarzenia:', end=' ')
        zd = int(input()) - 1

    t = tranzycje[zd]

    # Automatyczne wybieranie tranzycji
    # if master_on:
    #     t = master_transitions[path[i]]
    # else:
    #     t = slave_transitions[path[i]]
    # print("Tranzycja: ", events[t.identifier], '->', t.destinations[0].name)
    # i += 1
    # if i >= len(path):
    #     exit()

    # Wykonanie tranzycji
    t._run(state_machine)

    if t.identifier == 's_1_4' or t.identifier == 's_2_4' or t.identifier == 's_3_4':
        print('--------------------------------')
        print('Aktualny stan:', state_machine.current_state.name)
        print('Koniec podprocesu.')
        slave = Generator.create_master(slave_states, slave_transitions)
        master_transitions['m_4_1']._run(master)
        master_on = True

    print('--------------------------------')

    if t.identifier == 'm_1_4':
        print('Przejście do podprocesu.')
        master_on = False
