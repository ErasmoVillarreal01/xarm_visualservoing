Copiar adentro del paquete xarm_planner del repo de xarm


roslaunch xarm_planner xarm_planner_rviz_sim.launch robot_dof:=6 robot_type:=xarm  add_gripper:=true
roslaunch visp_auto_tracker tracklive_usb2.launch 
rosrun xarm_planner eff_pos.py
rosrun xarm_planner publisher.py 

