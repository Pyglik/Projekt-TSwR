from statemachine import StateMachine, State


class RobotMobilny(StateMachine):
    bezczynny = State('Robot bezczynny', initial=True)
    przod = State('Jazda do przodu')
    lewo = State('Skręcanie w lewo')
    prawo = State('Skręcanie w prawo')
    omijanie = State('Omijanie przeszkody')

    start = bezczynny.to(przod)
    stop = przod.to(bezczynny)
    sk_lewo = przod.to(lewo)
    sk_prawo = przod.to(prawo)
    koniec_sk_l = lewo.to(przod)
    koniec_sk_p = prawo.to(przod)
    przeszkoda = przod.to(omijanie)
    koniec_om = omijanie.to(przod)

    zdarzenia = {'start': 'Start',
                 'stop': 'Stop',
                 'sk_lewo': 'Skręć w lewo',
                 'sk_prawo': 'Skręć w prawo',
                 'koniec_sk_l': 'Koniec skręcania',
                 'koniec_sk_p': 'Koniec skręcania',
                 'przeszkoda': 'Przed robotem wykryto przeszkodę',
                 'koniec_om': 'Koniec omijania'}


class Omijanie(StateMachine):
    omijanie_pr = State('Omijanie przeszkody', initial=True)
    lewo = State('Skręcanie w lewo')
    prawo = State('Skręcanie w prawo')
    zawracanie = State('Zawracanie')
    koniec_om = State('Koniec omijania')

    prz_prawo = omijanie_pr.to(lewo)
    prz_lewo = omijanie_pr.to(prawo)
    prz_przod = omijanie_pr.to(prawo)
    prz_prawo_lewo = omijanie_pr.to(zawracanie)
    koniec_sk_l = lewo.to(koniec_om)
    koniec_sk_p = prawo.to(koniec_om)
    koniec_zaw = zawracanie.to(koniec_om)

    zdarzenia = {'prz_prawo': 'Przeszkoda wykryta z prawej',
                 'prz_lewo': 'Przeszkoda wykryta z lewej',
                 'prz_przod': 'Przeszkoda wykryta tylko z przodu',
                 'prz_prawo_lewo': 'Przeszkoda wykryta z obu stron',
                 'koniec_sk_l': 'Koniec skręcania',
                 'koniec_sk_p': 'Koniec skręcania',
                 'koniec_zaw': 'Koniec zawracania'}


def symulacja(state_machine):
    while True:
        print('Aktualny stan:', state_machine.current_state.name)
        tranzycje = state_machine.current_state.transitions

        if state_machine.current_state.identifier == 'omijanie':
            print('Przejście do podprocesu.')
            omijanie_pr = Omijanie()
            symulacja(omijanie_pr)

        if not tranzycje:
            print('Koniec pracy automatu.')
            return

        print('Dostępne tranzycje:')
        for i in range(len(tranzycje)):
            print(str(i+1)+'.', state_machine.zdarzenia[tranzycje[i].identifier],
                  '->', tranzycje[i].destinations[0].name)

        print('Wybierz zdarzenie:', end=' ')
        zd = int(input())-1
        while zd not in range(len(tranzycje)):
            print('Niepoprawne zdarzenie.')
            print('Podaj numer zdarzenia:', end=' ')
            zd = int(input())-1

        state_machine.run(tranzycje[zd].identifier)
        print('--------------------------------')


robot = RobotMobilny()
# print(robot.current_state)
# print(robot.current_state == robot.bezczynny)
# print(robot.is_bezczynny)
#
# print([s.identifier for s in robot.states])
# print([t.identifier for t in robot.transitions])
#
# robot.start()
# print(robot.current_state)
#
# print(robot.current_state.name)
# print([t.destinations[0].name for t in robot.current_state.transitions])

symulacja(robot)
