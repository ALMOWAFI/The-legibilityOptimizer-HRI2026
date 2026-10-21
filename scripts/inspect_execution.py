#!/usr/bin/env python3
"""Read-only capture of model fingerprint, collision IDs and controller metadata."""
import argparse
import json
import rospy
from controller_manager_msgs.srv import ListControllers
from moveit_msgs.msg import PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene, GetPlanningSceneRequest
from legibility_execution.ros_node import bounded, digest, ros_data


def main():
    rospy.init_node('inspect_legibility_execution', anonymous=True)
    parser = argparse.ArgumentParser()
    parser.add_argument('--robot-description', default='/robot_description')
    parser.add_argument('--controller-manager', default='/controller_manager/list_controllers')
    parser.add_argument('--scene-service', default='/get_planning_scene')
    args = parser.parse_args(rospy.myargv()[1:])
    prefix = args.robot_description
    model = dict(urdf=rospy.get_param(prefix), srdf=rospy.get_param(prefix + '_semantic'),
                 limits=rospy.get_param(prefix + '_planning/joint_limits'),
                 kinematics=rospy.get_param(prefix + '_kinematics'))
    request = GetPlanningSceneRequest()
    request.components.components = PlanningSceneComponents.WORLD_OBJECT_NAMES | PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
    scene = bounded(lambda: rospy.ServiceProxy(args.scene_service, GetPlanningScene)(request)).scene
    controllers = bounded(lambda: rospy.ServiceProxy(args.controller_manager, ListControllers)()).controller
    print(json.dumps(dict(commissioned_model_sha256=digest(model),
                          world_collision_objects=[ros_data(o) for o in scene.world.collision_objects],
                          controllers=[ros_data(c) for c in controllers]), indent=2, sort_keys=True))
    print('Read-only capture. The model hash is identification, not physical validation.')


if __name__ == '__main__':
    main()
