from statemachine import State
from generator import Generator, create_transitions

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

# # create paths from transitions (exemplary)
# path_1 = ['m_0_1', 'm_1_2', 'm_2_1', 'm_1_3', 'm_3_4']
# path_2 = ['m_0_2', 'm_2_3', 'm_3_2', 'm_2_4']
# path_3 = ['m_0_3', 'm_3_1', 'm_1_2', 'm_2_4']
# paths = [path_1, path_2, path_3]
#
# # execute paths
# for path in paths:
#
#     # create a supervisor
#     supervisor = Generator.create_master(master_states, master_transitions)
#     print('\n' + str(supervisor))
#
#     # run supervisor for exemplary path
#     print('Executing path: {}'.format(path))
#     for event in path:
#
#         # launch a transition in our supervisor
#         master_transitions[event]._run(supervisor)
#         print(supervisor.current_state)
#
#         # add slave
#         if supervisor.current_state.value == 'a':
#             # TODO: automata 1 (for) slave1
#             ...
#
#         if supervisor.current_state.value == 'b':
#             # TODO: automata 2 (for) slave2
#             ...
#
#         if supervisor.current_state.value == 'c':
#             # TODO: automata 3 (for) slave3
#             ...
#
#         if supervisor.current_state.value == 'f':
#             # TODO: automata 3 (for) slave3
#             ...
#             print('Supervisor done!')

master = Generator.create_master(master_states, master_transitions)
slave = Generator.create_master(slave_states, slave_transitions)

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

    print('Dostępne tranzycje:')
    for i in range(len(tranzycje)):
        print(str(i + 1) + '.', events[tranzycje[i].identifier],
              '->', tranzycje[i].destinations[0].name)

    print('Wybierz zdarzenie:', end=' ')
    zd = int(input()) - 1
    while zd not in range(len(tranzycje)):
        print('Niepoprawne zdarzenie.')
        print('Podaj numer zdarzenia:', end=' ')
        zd = int(input()) - 1

    tranzycje[zd]._run(state_machine)

    if tranzycje[zd].identifier == 's_1_4' or tranzycje[zd].identifier == 's_2_4'\
            or tranzycje[zd].identifier == 's_3_4':
        print('--------------------------------')
        print('Aktualny stan:', state_machine.current_state.name)
        print('Koniec podprocesu.')
        slave = Generator.create_master(slave_states, slave_transitions)
        master_transitions['m_4_1']._run(master)
        master_on = True

    print('--------------------------------')

    if tranzycje[zd].identifier == 'm_1_4':
        print('Przejście do podprocesu.')
        master_on = False
