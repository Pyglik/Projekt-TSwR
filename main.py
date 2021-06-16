from statemachine import State
from pynput import keyboard
from generator import Generator, create_transitions
from graph import draw_graph
from mir_controler import MirControler

key = None


def on_press(k):
    global key
    try:
        key = k.char
    except AttributeError:
        pass


def check_recurrence(cur_state, end_state, to_visit):
    if cur_state == end_state:
        return []
    to_visit.remove(cur_state)
    if not to_visit:
        return None

    output = None
    for t in cur_state.transitions:
        next_s = t.destinations[0]
        if next_s not in to_visit:
            continue
        out = check_recurrence(next_s, end_state, to_visit.copy())
        if out is not None and (output is None or len(out)+1 < len(output)):
            output = out
            output.append(t)

    return output


def check_state_machine(state_machine, start_state, end_state):
    if start_state not in state_machine.states or end_state not in state_machine.states:
        print('Co najmniej jeden z podanych stanów nie należy do automatu!')
        return None
    to_visit = state_machine.states
    output = check_recurrence(start_state, end_state, to_visit.copy())
    if output is None:
        print('Brak połączenia między podanymi stanami!')
        return None
    output = [t.identifier for t in output]
    output.reverse()
    return output


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

print(check_state_machine(master, master_states[0], master_states[3]))
print(check_state_machine(slave, slave_states[0], slave_states[4]))
print(check_state_machine(master, master_states[1], slave_states[3]))
print(check_state_machine(slave, slave_states[1], slave_states[2]))

mir_con = MirControler()

listener = keyboard.Listener(on_press=on_press)
listener.start()

i = 0
master_on = True
while True:
    if master_on:
        state_machine = master
        events = master_events
    else:
        state_machine = slave
        events = slave_events

    print('--------------------------------')
    print('Aktualny stan:', state_machine.current_state.name)
    tranzycje = state_machine.allowed_transitions

    id = state_machine.states.index(state_machine.current_state)
    if master_on:
        state = 'm'+str(id)
    else:
        state = 's'+str(id)
    draw_graph(state)

    # --------------------------------
    # # Ręczne wybieranie tranzycji
    # print('Dostępne tranzycje:')
    # for i in range(len(tranzycje)):
    #     print(str(i + 1) + '.', events[tranzycje[i].identifier], '->', tranzycje[i].destinations[0].name)
    #
    # print('Wybierz zdarzenie:', end=' ')
    # zd = int(input()) - 1
    # while zd not in range(len(tranzycje)):
    #     print('Niepoprawne zdarzenie.')
    #     print('Podaj numer zdarzenia:', end=' ')
    #     zd = int(input()) - 1
    #
    # t = tranzycje[zd]
    # --------------------------------
    # # Automatyczne wybieranie tranzycji
    # if master_on:
    #     t = master_transitions[path[i]]
    # else:
    #     t = slave_transitions[path[i]]
    # print("Tranzycja: ", events[t.identifier], '->', t.destinations[0].name)
    # i += 1
    # if i >= len(path):
    #     exit()
    # --------------------------------
    # # Symulacja
    # sterowanie robotem
    if state == 'm0':  # Robot bezczynny
        mir_con.idle()
    elif state == 'm1':  # Jazda do przodu
        mir_con.drive()
    elif state == 'm2':  # Skręcanie w lewo
        while not mir_con.turn_left_end():
            mir_con.turn_left()
    elif state == 'm3':  # Skręcanie w prawo
        while not mir_con.turn_right_end():
            mir_con.turn_right()
    elif state == 's1':  # Skręcanie w lewo
        while not mir_con.turn_left_end():
            mir_con.turn_left()
    elif state == 's2':  # Skręcanie w prawo
        while not mir_con.turn_right_end():
            mir_con.turn_right()
    elif state == 's3':  # Zawracanie
        while not mir_con.turn_around_end():
            mir_con.turn_around()

    t = None

    # # tranzycje z klawiatury
    # if key == 'q':  # zatrzymanie symulacji
    #     exit(0)
    # elif master_on and key == 'w':  # Start
    #     t = master_transitions['m_0_1']
    # elif master_on and key == 's':  # Stop
    #     t = master_transitions['m_1_0']
    # elif master_on and key == 'a':  # Skręć w lewo
    #     t = master_transitions['m_1_2']
    # elif master_on and key == 'd':  # Skręć w prawo
    #     t = master_transitions['m_1_3']
    # key = None

    # tranzycje z robota
    if not master_on and mir_con.turn_around_end():  # Koniec zawracania
        t = slave_transitions['s_3_4']
    elif master_on and mir_con.turn_left_end():  # Koniec skręcania
        t = master_transitions['m_2_1']
    elif master_on and mir_con.turn_right_end():  # Koniec skręcania
        t = master_transitions['m_3_1']
    elif not master_on and mir_con.turn_left_end():  # Koniec skręcania
        t = slave_transitions['s_1_4']
    elif not master_on and mir_con.turn_right_end():  # Koniec skręcania
        t = slave_transitions['s_2_4']
    elif master_on and id == 1 and mir_con.obstacle():  # Przed robotem wykryto przeszkodę
        t = master_transitions['m_1_4']
    elif not master_on and mir_con.obstacle_r():  # Przeszkoda wykryta z prawej
        t = slave_transitions['s_0_1']
    elif not master_on and mir_con.obstacle_lf():  # Przeszkoda wykryta z lewej lub z przodu
        t = slave_transitions['s_0_2']
    elif not master_on and mir_con.obstacle_lr():  # Przeszkoda wykryta z obu stron
        t = slave_transitions['s_0_3']

    if t is not None:
        print('Automatyczna tranzycja: ', events[t.identifier], '->', t.destinations[0].name)
        input('Wciśnij Enter...')
    else:  # tranzycje ręczne
        print('Dostępne tranzycje:')
        for i in range(len(tranzycje)):
            print(str(i + 1) + '.', events[tranzycje[i].identifier], '->', tranzycje[i].destinations[0].name)
        print(str(len(tranzycje) + 1) + '. Pozostań w aktualnym stanie')

        print('Wybierz zdarzenie:', end=' ')
        inp = input()
        if inp == 'q':
            exit(0)
        zd = int(inp) - 1
        while zd not in range(len(tranzycje) + 1):
            print('Niepoprawne zdarzenie.')
            print('Podaj numer zdarzenia:', end=' ')
            inp = input()
            if inp == 'q':
                exit(0)
            zd = int(inp) - 1

        if zd != len(tranzycje):
            t = tranzycje[zd]
    # --------------------------------

    # Wykonanie tranzycji
    if t in tranzycje:
        t._run(state_machine)

        if t.identifier == 's_1_4' or t.identifier == 's_2_4' or t.identifier == 's_3_4':
            print('--------------------------------')
            print('Aktualny stan:', state_machine.current_state.name)
            print('Koniec podprocesu.')
            slave = Generator.create_master(slave_states, slave_transitions)
            master_transitions['m_4_1']._run(master)
            master_on = True

        if t.identifier == 'm_1_4':
            print('Przejście do podprocesu.')
            master_on = False
