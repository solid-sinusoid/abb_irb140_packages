# abb_irb140_packages
## Общее

Привет, данный репозиторий разработан в рамках выполнения выпускной квалификационной работы бакалавра на кафедре "Мехатронные системы" в университете ИжГТУ имени М.Т. Калашникова

### Данный репозиторий включате такие пакеты как:

- [MoveIt][moveit_install]
- [abb_driver][abb_d_repo]
- [IKFast][ikfast]
- [descartes][descartes]
- [ROS Industrial Core][ros_i_core]

Данный репозиторий протестирован на реальном промышленном роботе манипуляторе который расположен в лаборатории 3-403а. Дальнейшее развитие проекта планируется в магистратуре, где будет добавлена система сканирования окружения и работы с ним.

## Использование репозитория
### Установка
Для установки необходимо иметь Ubuntu 18.04, ROS Melodic, MoveIt и QTCreator 4 с установленным ROS-I плагином

Команда установки:
```sh
$ git clone https://github.com/solid-sinusoid/abb_irb140_packages.git
```
Для установки остальных пакетов, связанных зависимостью с данным пакетом существует команда `rosdep` 

```
$ rosdep update
$ rosdep install --from-paths src / --ignore-src --rosdistro melodic
$ catkin_make
```

### Запуск
Для запуска Rviz используется следующая команда:
```sh
$ roslaunch abb_new_moveit_config demo.launch
```
Для запуска симуляции сначала необходимо запустить Gazebo:
```sh
$ roslaunch abb_irb140_gazebo irb140_gazebo.launch
```
И следом идёт запуск визуализации
```sh
$ roslaunch abb_new_moveit_config abb_new_planing_execution.launch
```
И сразу же **остановить** запуск второго и запустить заново

Для запуска на реальном роботе манипуляторе: 
```sh
$ roslaunch abb_new_moveit_config moveit_planing_execution sim:=false robot_ip:=111.111.111.1
```
Переменная `robot_ip` указана на экране планшета робота.

В данном случа всё должно заработать сразу. Чтобы задаваемые программы исполнялись необходимо включить автоматический режим на роботе и запустить основную программу

[moveit_install]: <https://moveit.ros.org/install/>
[abb_d_repo]: <https://github.com/ros-industrial/abb_driver>
[ikfast]: <http://docs.ros.org/en/melodic/api/moveit_tutorials/html/doc/ikfast/ikfast_tutorial.html>
[descartes]: <https://github.com/ros-industrial-consortium/descartes>
[ros_i_core]: <https://github.com/ros-industrial/industrial_core>

