#!/usr/bin/env python3

import sys
import copy
import rospy
import moveit_commander
import geometry_msgs.msg
from tf.transformations import quaternion_from_euler


def move_cartesian(move_group, waypoints):
    """Auxiliar para executar movimentos retilíneos espaciais no referencial da base."""
    (plan, fraction) = move_group.compute_cartesian_path(
        waypoints,   # Lista de pontos de passagem (geometry_msgs.msg.Pose)
        0.01,        # Passo de amostragem (1 cm)
        0.0          # Limite de salto (evita descontinuidades de IK)
    )
    if fraction > 0.9:  # Garante que mais de 90% do caminho foi planejado com sucesso
        move_group.execute(plan, wait=True)
        move_group.stop()
        return True
    else:
        rospy.logerr("Falha ao planejar trajetória cartesiana retilínea.")
        return False


def main():
    # 1. Inicialização do nó ROS e MoveIt
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("ur_pick_and_place_base", anonymous=True)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    group_name = "manipulator"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    # ----------------------------------------------------
    # CONFIGURAÇÃO DO REFERENCIAL DA BASE
    # ----------------------------------------------------
    reference_frame = "base_link"  # Define a base do UR como referencial global
    move_group.set_pose_reference_frame(reference_frame)
    
    # Tolerâncias e Velocidades
    move_group.set_goal_position_tolerance(0.005)
    move_group.set_goal_orientation_tolerance(0.02)
    move_group.set_max_velocity_scaling_factor(0.2)
    move_group.set_max_acceleration_scaling_factor(0.2)

    rospy.sleep(1)

    # Conversão de orientação: Garra apontada para baixo (Roll = 180°, Pitch = 0°, Yaw = 0°)
    q = quaternion_from_euler(3.14159, 0, 0)

    rospy.loginfo(f"Iniciando ciclo referente à frame: {move_group.get_planning_frame()}")

    # ----------------------------------------------------
    # PASSO 1: Ir até a Posição Home (Juntas)
    # ----------------------------------------------------
    rospy.loginfo("Movendo para a posição de espera (Home)...")
    joint_goal = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]
    move_group.go(joint_goal, wait=True)
    move_group.stop()

    # ----------------------------------------------------
    # PASSO 2: Aproximação e Pick (Referente ao base_link)
    # ----------------------------------------------------
    # Posição de Aproximação (Pick) - 10cm acima
    pose_pick_approach = geometry_msgs.msg.Pose()
    pose_pick_approach.orientation.x = q[0]
    pose_pick_approach.orientation.y = q[1]
    pose_pick_approach.orientation.z = q[2]
    pose_pick_approach.orientation.w = q[3]
    pose_pick_approach.position.x = -0.18963
    pose_pick_approach.position.y = -0.60968
    pose_pick_approach.position.z = 0.36097

    rospy.loginfo("Movendo para aproximação do Pick...")
    move_group.set_pose_target(pose_pick_approach)
    move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()

    # Descida cartesiana em linha reta Z (Pick)
    rospy.loginfo("Descendo em linha reta para o Pick...")
    pose_pick = copy.deepcopy(pose_pick_approach)
    pose_pick.position.z = 0.26097  # Posição final de pega
    
    move_cartesian(move_group, [pose_pick])

    # --- SIMULAÇÃO DA GARRA ---
    rospy.loginfo("Objeto agarrado (Simulado).")
    rospy.sleep(1.0)

    # Subida cartesiana em linha reta Z
    rospy.loginfo("Subindo em linha reta com o objeto...")
    move_cartesian(move_group, [pose_pick_approach])

    # ----------------------------------------------------
    # PASSO 3: Transporte e Place (Referente ao base_link)
    # ----------------------------------------------------
    # Defina aqui as coordenadas de destino do Place (em relação ao base_link)
    pose_place_approach = copy.deepcopy(pose_pick_approach)
    pose_place_approach.position.x = 0.30000  # Exemplo: movendo o objeto no eixo X
    pose_place_approach.position.y = -0.50000

    rospy.loginfo("Transportando para o local de soltura...")
    move_group.set_pose_target(pose_place_approach)
    move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()

    # Descida cartesiana em linha reta (Place)
    rospy.loginfo("Descendo para o Place...")
    pose_place = copy.deepcopy(pose_place_approach)
    pose_place.position.z = 0.26097
    
    move_cartesian(move_group, [pose_place])

    # --- SIMULAÇÃO DA GARRA ---
    rospy.loginfo("Objeto solto (Simulado).")
    rospy.sleep(1.0)

    # Recuo cartesiano para cima
    rospy.loginfo("Subindo após soltar...")
    move_cartesian(move_group, [pose_place_approach])

    # ----------------------------------------------------
    # PASSO 4: Retorno ao Home
    # ----------------------------------------------------
    rospy.loginfo("Retornando para a posição Home...")
    move_group.go(joint_goal, wait=True)
    move_group.stop()

    rospy.loginfo("Ciclo concluído com sucesso!")


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass