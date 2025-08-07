#!/usr/bin/env python3

import rclpy
import yaml
import os
import time
from typing import Dict, Any

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

import yasmin
from yasmin import State, StateMachine, Blackboard


class InitNavigationState(State):
    
    def __init__(self):
        super().__init__(["success", "error"])
        
    def execute(self, blackboard: Dict[str, Any]) -> str:
        try:
            navigator = BasicNavigator()
            
            yasmin.YASMIN_LOG_INFO("Esperando a que Nav2 esté activo...")
            navigator.waitUntilNav2Active()
            yasmin.YASMIN_LOG_INFO("Nav2 está activo!")
            
            waypoints_path = os.path.join(
                get_package_share_directory('yasmin_waypoint_nav'),
                'config',
                'waypoints.yaml'
            )
            
            with open(waypoints_path, 'r') as f:
                waypoints = yaml.safe_load(f)
            
            blackboard["navigator"] = navigator
            blackboard["waypoints"] = waypoints
            blackboard["current_waypoint_index"] = 0
            blackboard["total_waypoints"] = len(waypoints)
            
            yasmin.YASMIN_LOG_INFO(f"Waypoints cargados: {len(waypoints)}")
            
            return "success"
            
        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"Error en inicialización: {str(e)}")
            return "error"


class LoadNextWaypointState(State):
    
    def __init__(self):
        super().__init__(["waypoint_loaded", "all_waypoints_completed"])
        
    def execute(self, blackboard: Dict[str, Any]) -> str:
        current_index = blackboard["current_waypoint_index"]
        total_waypoints = blackboard["total_waypoints"]
        
        if current_index >= total_waypoints:
            yasmin.YASMIN_LOG_INFO("Todos los waypoints han sido completados!")
            return "all_waypoints_completed"
        
        waypoints = blackboard["waypoints"]
        current_wp = waypoints[current_index]
        
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.pose.position.x = current_wp['x']
        goal_pose.pose.position.y = current_wp['y']
        goal_pose.pose.orientation.z = current_wp['z']
        goal_pose.pose.orientation.w = current_wp['w']
        
        blackboard["current_goal_pose"] = goal_pose
        
        yasmin.YASMIN_LOG_INFO(f"Waypoint {current_index + 1}/{total_waypoints} cargado: "
                              f"x={current_wp['x']}, y={current_wp['y']}")
        
        return "waypoint_loaded"


class NavigateToWaypointState(State):
    
    def __init__(self):
        super().__init__(["navigation_success", "navigation_failed"])
        
    def execute(self, blackboard: Dict[str, Any]) -> str:
        navigator = blackboard["navigator"]
        goal_pose = blackboard["current_goal_pose"]
        current_index = blackboard["current_waypoint_index"]
        
        yasmin.YASMIN_LOG_INFO(f"Navegando al waypoint {current_index + 1}...")
        
        navigator.goToPose(goal_pose)
        
        while not navigator.isTaskComplete():
            feedback = navigator.getFeedback()
            if feedback:
                yasmin.YASMIN_LOG_INFO(f"Distancia restante al WP {current_index + 1}: "
                                      f"{feedback.distance_remaining:.2f} m")
            time.sleep(1)
        
        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            yasmin.YASMIN_LOG_INFO(f"Objetivo {current_index + 1} alcanzado con éxito!")
            return "navigation_success"
        else:
            yasmin.YASMIN_LOG_ERROR(f"Fallo al alcanzar el objetivo {current_index + 1}")
            return "navigation_failed"


class UpdateWaypointIndexState(State):
    
    def __init__(self):
        super().__init__(["updated"])
        
    def execute(self, blackboard: Dict[str, Any]) -> str:
        blackboard["current_waypoint_index"] += 1
        yasmin.YASMIN_LOG_INFO("Índice de waypoint actualizado")
        return "updated"


class WaypointReachedState(State):
    
    def __init__(self):
        super().__init__(["continue"])
        
    def execute(self, blackboard: Dict[str, Any]) -> str:
        current_index = blackboard["current_waypoint_index"]
        yasmin.YASMIN_LOG_INFO(f"Waypoint {current_index + 1} alcanzado. Procesando...")
        
        time.sleep(2)
        
        return "continue"


class FinishState(State):
    
    def __init__(self):
        super().__init__(["finished"])
        
    def execute(self, blackboard: Dict[str, Any]) -> str:
        yasmin.YASMIN_LOG_INFO("¡Navegación por waypoints completada exitosamente!")
        return "finished"


class ErrorState(State):
    
    def __init__(self):
        super().__init__(["error"])
        
    def execute(self, blackboard: Dict[str, Any]) -> str:
        yasmin.YASMIN_LOG_ERROR("Error en la navegación por waypoints")
        return "error"


def create_waypoint_state_machine():
    
    sm = StateMachine(outcomes=["success", "error"])
    
    sm.add_state("INIT_NAVIGATION", InitNavigationState(),
                 transitions={"success": "LOAD_NEXT_WAYPOINT",
                             "error": "ERROR"})
    
    sm.add_state("LOAD_NEXT_WAYPOINT", LoadNextWaypointState(),
                 transitions={"waypoint_loaded": "NAVIGATE_TO_WAYPOINT",
                             "all_waypoints_completed": "FINISH"})
    
    sm.add_state("NAVIGATE_TO_WAYPOINT", NavigateToWaypointState(),
                 transitions={"navigation_success": "WAYPOINT_REACHED",
                             "navigation_failed": "ERROR"})
    
    sm.add_state("WAYPOINT_REACHED", WaypointReachedState(),
                 transitions={"continue": "UPDATE_WAYPOINT_INDEX"})
    
    sm.add_state("UPDATE_WAYPOINT_INDEX", UpdateWaypointIndexState(),
                 transitions={"updated": "LOAD_NEXT_WAYPOINT"})
    
    sm.add_state("FINISH", FinishState(),
                 transitions={"finished": "success"})
    
    sm.add_state("ERROR", ErrorState(),
                 transitions={"error": "error"})
    
    return sm


def main():
    
    try:
        rclpy.init()
        
        sm = create_waypoint_state_machine()
        
        blackboard = Blackboard()
        
        yasmin.YASMIN_LOG_INFO("Iniciando navegación por waypoints con YASMIN...")
        outcome = sm.execute(blackboard)
        yasmin.YASMIN_LOG_INFO(f"Navegación terminada con resultado: {outcome}")
        
    except KeyboardInterrupt:
        yasmin.YASMIN_LOG_INFO("Navegación interrumpida por el usuario")
    except Exception as e:
        yasmin.YASMIN_LOG_ERROR(f"Error durante la ejecución: {str(e)}")
        import traceback
        traceback.print_exc()
    finally:
        try:
            rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main()
