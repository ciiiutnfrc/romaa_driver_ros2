# Paquete `romaa_driver` para ROS 2

Paquete con nodo driver.

## Nodo `romaa_driver`
Nodo driver para el controlador embebido de tracción del robot móvil RoMAA-II.

### Subcriptores
- `cmd_vel` (`geometry_msgs/msg/Twist`): comandos de velocidad.

### Publicadores
- `odom` (`nav_msgs/msg/Odometry`): información de odometría.
- `tf` (`tf2_msgs/msg/TFMessage`): publica la transformación de `odom_frame` a `base_frame`.
