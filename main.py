from statemachine import StateMachine, State


class MobileRobot(StateMachine):
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


robot = MobileRobot()
print(robot.current_state)
print(robot.current_state == robot.bezczynny)
print(robot.is_bezczynny)

print([s.identifier for s in robot.states])
print([t.identifier for t in robot.transitions])

robot.start()
print(robot.current_state)
