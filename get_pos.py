#!/usr/bin/env python
import sys
import rospy
import moveit_commander

def print_current_pose():
    # Inicializa o moveit_commander e o nó ROS
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('get_robot_pose', anonymous=True)

    # Substitua 'manipulator' pelo nome do seu grupo no MoveIt
    group_name = "manipulator"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    # Obtém a pose atual do End-Effector (TCP)
    current_pose = move_group.get_current_pose().pose

    print("--- POSIÇÃO ATUAL DO ROBÔ ---")
    print("Frame de referência:", move_group.get_planning_frame())
    print("Posição (metros):")
    print("  X: {:.5f}".format(current_pose.position.x))
    print("  Y: {:.5f}".format(current_pose.position.y))
    print("  Z: {:.5f}".format(current_pose.position.z))
    print("Orientação (Quaternião):")
    print("  X: {:.5f}, Y: {:.5f}, Z: {:.5f}, W: {:.5f}".format(
        current_pose.orientation.x,
        current_pose.orientation.y,
        current_pose.orientation.z,
        current_pose.orientation.w
    ))

    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try:
        print_current_pose()
    except rospy.ROSInterruptException:
        pass