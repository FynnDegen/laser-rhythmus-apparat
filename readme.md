# laser-rhythmus-apparat ros2 (lra)

## Vorraussetzungen

- Linux Ubuntu

## Installation

```
sudo apt-get install curl cmake clang pulseaudio libasound-dev libjack-dev
```

## Starten

- Konsole starten
- zwei weitere Tabs mit `Strg + t` öffnen

- Simulation starten

```
ros2 launch ceres_gazebo ceres_gazebo_launch.py
```

- lra starten

```
ros2 run lra subscriber_class
```

- Robotersteuerung starten

```
ros2 launch uos_diffdrive_teleop key.launch
```