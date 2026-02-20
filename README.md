# Paquete `romaa_driver` para ROS 2

Paquete con nodo driver.

## Nodo `romaa_driver`
Nodo driver para el controlador embebido de tracción del robot móvil RoMAA-II.

### Subcriptores
- `cmd_vel` (`geometry_msgs/msg/Twist`): comandos de velocidad.

### Publicadores
- `odom` (`nav_msgs/msg/Odometry`): información de odometría.
- `tf` (`tf2_msgs/msg/TFMessage`): publica la transformación de `odom_frame` a `base_frame`.

### Parámetros estáticos
- `frequency` (`double`, default: `10.0`): frecuencia de publicación del nodo.
- `port` (`string`, default: `/dev/ttyUSB0`): archivo de dispositivo de comunicación.
- `baudrate` (`int`, default: `115200`): velocidad de la comunicación.
- `odom_frame` (`string`, default: `odom`): nombre del frame de odometría.
- `base_frame` (`string`, default: `base_link`): nombre del frame del robot.
- `enable_motor` (`bool`, default: `false`): inidica habilitar los motores al inicio de la ejecución.
- `reset_odom` (`bool`, default: `false`): indica resetear la odometría al inicio de la ejecución.

### Servicios
- `reset` (`std_srvs/srv/Empty`): reinicia el controlador embebido.
- `reset_odometry` (`std_srvs/srv/Empty`): reinicia la odometría fijando el valor de pose a (0, 0, 0).
- `enable_motor` (`std_srvs/srv/SetBool`): habilita/deshabilita los motores de tracción del controlador embebido.
- `set_odometry` (`romaa_driver_interfaces/srv/SetOdometry`): modifica el valor de odometría (x, y, theta).
