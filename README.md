# Repositorio del TFG de Unai Sanz [@USanz](https://github.com/USanz)

En este repositorio se subirán todos los archivos y código relacionado con mi TFG, que trata de resolver los problemas que se encuentran a la hora de automatizar robots de bajo coste, como pueden ser la localización, navegación, etc. Concretamente se enfoca en ámbitos de ayuda o asistencia personal en supermercados, pero puede aplicarse a otro tipo de situaciones y lugares.

## Mas información

Para más información sobre el proyecto visitar el apartado de la [wiki](https://github.com/RoboticsURJC/tfg-unai/wiki)

## Instrucciones de instalación

Este proyecto ha sido probado usando ROS2, distribución Foxy Fitzroy, en un sistema GNU/Linux, Ubuntu versión 20.04.1, arquitectura x86 64 bits.

### Prerrequisitos:

[ROS2 Foxy Fitzroy](https://docs.ros.org/en/foxy/Installation.html)

[Gazebo simulator](https://osrf.github.io/gazebo-doc-index/categories/installing_gazebo.html)

RViz2 (Debería haber sido instalado durante la instalación de ROS2)

<!--
[OpenCV library for c++](https://docs.opencv.org/4.x/d7/d9f/tutorial_linux_install.html)
-->

### Ejecución:

Para ejecutar la simulación con el mundo de gazebo:

```
ros2 launch pibot_robot world.launch.py
```

Para ejecutar la simulación con el mundo de gazebo y el robot:

```
ros2 launch pibot_robot robot_sdf.launch.py
```
