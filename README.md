
# Repositorio del TFG de [Unai Sanz](https://github.com/USanz)

En este repositorio se subirán todos los archivos y código relacionado con mi
TFG, que trata de resolver el problema de la programación de flujos de datos en
robótica, aplicando para ello nuevas tecnologías como [Zenoh-flow](https://github.com/eclipse-zenoh/zenoh-flow) y nuevos
protocolos de comunicaciones como [Zenoh](https://zenoh.io/). Concretamente se enfoca en un ámbito
multirobot tanto simulado como real.



## Mas información

Para más información sobre el proyecto visitar el apartado de la
[wiki](https://github.com/RoboticsURJC/tfg-unai/wiki), donde se explica más a fondo en cada una de sus páginas los distintos aspectos, problemas y soluciones llevados a cabo durante su desarrollo.



## Instrucciones de instalación

Este proyecto ha sido probado en un portátil y un araspberry con las siguientes especificaciones:

 - Portátil: ROS2 Humble Hawksbill, sistema GNU/Linux, Ubuntu versión 22.04.1, arquitectura x86 de 64 bits (AMD64).

 - Raspberry Pi 4b (1Gb de RAM + 2 de Swap): ROS2 Humble Hawksbill, sistema GNU/Linux, Ubuntu Server versión 22.04.1, con una arquitectura aarch de 64 bits ( ARM64).



### Prerrequisitos:

 - Exclusivamente en el portátil:

    - [Gazebo simulator](https://osrf.github.io/gazebo-doc-index/categories/installing_gazebo.html)

    - [Zenoh-flow](https://github.com/ZettaScaleLabs/zenoh-flow-examples/blob/master/transcoding/README.md) (tag: v0.5.0-alpha.1)

 - En ambos (tanto en el portátil como en la Raspberry Pi):

    - [ROS2 Humble Hawksbill](https://docs.ros.org/en/humble/Installation.html)

    - RViz2 (Debería haber sido instalado durante la instalación de ROS2)

    - [Nav2](https://github.com/ros-planning/navigation2/tree/1.1.8) (tag: 1.1.8)

    - [Zenoh](https://github.com/ZettaScaleLabs/zenoh-flow-examples/blob/master/transcoding/README.md) (versión v0.7.0-rc)

    - [Zenoh-bridge-dds](https://github.com/eclipse-zenoh/zenoh-plugin-dds#how-to-install-it) (versión v0.7.2-rc)



### Ejecución:

 - Ejecución en simulación (ver pasos en el [apartado "Layout de terminales y comandos de ejecución" de la página 11 de la wiki](https://github.com/RoboticsURJC/tfg-unai/wiki/11.-Mejoras-y-correcciones-del-swarm_obj_finder-%5B30-Sep-%E2%80%90-9-Nov%5D#layout-de-terminales-y-comandos-de-ejecuci%C3%B3n)).

 - Ejecución en robots reales (ver pasos en el [apartado "Pruebas con varios robots y modelos mixtos (Turtlebot2 y Turtlebot4)" de la página 14 de la wiki](https://github.com/RoboticsURJC/tfg-unai/wiki/14.-Pruebas-de-laboratorio-%5B27-Nov-%E2%80%90-Hoy%5D#pruebas-con-varios-robots-y-modelos-mixtos-turtlebot2-y-turtlebot4)).

### Documentación del proyecto:

 - [Wiki del proyecto](https://github.com/RoboticsURJC/tfg-unai/wiki).
 - [Lista de reproducción en YouTube de vídeos de las pruebas](https://www.youtube.com/watch?v=x8VAZFSAH1w&list=PL3OZgGkAPYdkHaZFa2naBTB4b5aglx1h4)