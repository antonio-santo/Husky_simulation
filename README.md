# Husky_simulation
1. Pasos:
  - Descargar los paquetes en tu workspace.
  - Realizar un catkin_make (si dan errores, no importa ya que proceden de librerías que ahora mismo no se están usando)
2. Para introducir Ouster Lidar en simulación, realizar desde una terminal:
  - export HUSKY_OUSTER_ENABLED=1
3. En caso de querer mover el sensor de la posición donde sale por defecto, se debe modifcar la siguiente secuencia del archivo husky.xacro.urdf
  -   <xacro:arg name="ouster_xyz" default="$(optenv HUSKY_OUSTER_XYZ -0.0812 0 0.35618)" />
