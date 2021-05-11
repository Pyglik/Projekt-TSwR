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
    koniec_sk_l = sk_lewo.to(przod)
    koniec_sk_p = sk_prawo.to(przod)
    przeszkoda = przod.to(omijanie)
    koniec_om = omijanie.to(przod)
