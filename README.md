# Projekt-TSwR
## Temat: Sterowanie prostym robotem mobilnym z automatycznym omijaniem przeszkód.
## Jakub Szczygieł
## Schemat procesu:
![Schemat](TSwR.png)
## Użycie:
Pobranie i zbudownie repozytoriów
```
cd catkin_ws/src
git clone https://github.com/Pyglik/Projekt-TSwR
git clone https://github.com/dfki-ric/mir_robot
cd ..
catkin_make
```
Uruchomienie symulacji (należy również kliknąć przycisk start w Gazebo)
```
roslaunch mir_gazebo mir_maze_world.launch
```
Uruchomienie programu do sterowania
```
cd src/Projekt-TSwR
python3 main.py
```
