#!/usr/bin/env python

import sys
import rospy
import moveit_commander
import geometry_msgs.msg
import tf.transformations as tf_trans

def normalize_quaternion(x, y, z, w):
    """Garante que a orientação enviada é um quaternião unitário válido."""
    q = [x, y, z, w]
    normalized_q = tf_trans.unit_vector(q)
    return normalized_q

def execute_poses():
    # 1. Inicializar ROS e MoveIt
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('moveit_quaternion_execution', anonymous=True)

    group_name = "manipulator"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    move_group.set_planning_time(10.0)
    move_group.set_max_velocity_scaling_factor(0.1)
    move_group.set_max_acceleration_scaling_factor(0.1)

    # Pausa inicial para estabilizar a comunicação das ações no ROS
    rospy.sleep(1.0)

    # --------------------------------------------------------------------------
    # 2. DEFINIR POSE 1
    # --------------------------------------------------------------------------
    pose_1 = geometry_msgs.msg.Pose()
    pose_1.position.x = 0.18655
    pose_1.position.y = 0.26363
    pose_1.position.z = 0.38783
    
    q1 = normalize_quaternion(-0.99839, -0.04375, -0.03440, 0.01077)
    pose_1.orientation.x = q1[0]
    pose_1.orientation.y = q1[1]
    pose_1.orientation.z = q1[2]
    pose_1.orientation.w = q1[3]

    # --------------------------------------------------------------------------
    # 3. DEFINIR POSE 2
    # --------------------------------------------------------------------------
    pose_2 = geometry_msgs.msg.Pose()
    pose_2.position.x = 0.18676
    pose_2.position.y = 0.26347
    pose_2.position.z = 0.32933
    
    q2 = normalize_quaternion(-0.99842, -0.04238, -0.03421, 0.01078)
    pose_2.orientation.x = q2[0]
    pose_2.orientation.y = q2[1]
    pose_2.orientation.z = q2[2]
    pose_2.orientation.w = q2[3]

    # --------------------------------------------------------------------------
    # 4. EXECUÇÃO DO PRIMEIRO PONTO (Pose 1 - Movimento PTP/Espaço de Juntas)
    # --------------------------------------------------------------------------
    rospy.loginfo("--- A planear para a Pose 1 ---")
    move_group.set_pose_target(pose_1)
    
    plan_result = move_group.plan()
    if isinstance(plan_result, tuple):
        plan_success = plan_result[0]
        plan_1 = plan_result[1]
    else:
        plan_1 = plan_result
        plan_success = len(plan_1.joint_trajectory.points) > 0

    if plan_success and len(plan_1.joint_trajectory.points) > 0:
        rospy.loginfo("Plano para a Pose 1 gerado. A executar...")
        exec_success = move_group.execute(plan_1, wait=True)
        move_group.clear_pose_targets()

        if not exec_success:
            rospy.logerr("O movimento para a Pose 1 falhou durante a execução.")
            return
    else:
        rospy.logerr("Não foi possível encontrar solução cinemática (IK) para a Pose 1.")
        return

    rospy.sleep(0.5)

    # --------------------------------------------------------------------------
    # 5. EXECUÇÃO DO SEGUNDO PONTO (Pose 2 - Trajetória Cartesiana)
    # --------------------------------------------------------------------------
    rospy.loginfo("--- A planear trajetória cartesiana para a Pose 2 ---")
    waypoints = [pose_2]
    
    # Chamada compatível universalmente: apenas waypoints e eef_step (0.001 m = 1 mm)
    (plan_cartesian, fraction) = move_group.compute_cartesian_path(
        waypoints,
        0.001
    )

    rospy.loginfo("Fração da trajetória cartesiana calculada: %.2f%%", fraction * 100)

    if fraction > 0.95:
        # Reparametrização de tempo para garantir velocidade/aceleração controladas
        plan_cartesian = move_group.retime_trajectory(
            move_group.get_current_state(),
            plan_cartesian,
            velocity_scaling_factor=0.1,
            acceleration_scaling_factor=0.1
        )

        rospy.loginfo("A executar movimento linear para a Pose 2...")
        exec_cartesian_success = move_group.execute(plan_cartesian, wait=True)

        if exec_cartesian_success:
            rospy.loginfo("Sequência concluída com sucesso!")
        else:
            rospy.logerr("A execução do movimento cartesiano para a Pose 2 falhou.")
    else:
        rospy.logwarn("Trajetória cartesiana incompleta (%.2f%%). Possível singularidade ou limite de juntas.", fraction * 100)

    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try:
        execute_poses()
    except rospy.ROSInterruptException:
        pass