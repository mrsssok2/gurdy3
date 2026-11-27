#!/usr/bin/env python3
"""For the RViz / MoveIt viewing session: (1) add the floor to MoveIt's planning scene, (2) trigger the wrist camera at
TRIGGER_HZ so the RViz image panel updates (the camera renders only when triggered).

    python3 scripts/moveit_view_extras.py [floor_z_in_base_link] [trigger_hz]      (defaults -0.107 m, 2 Hz)
Floor z: the wrist-camera floor fit of day_0927 step 09 (-0.107 m in base_link; robot.yaml geometry says about -0.11).
The floor is a thin box (4 x 4 x 0.01 m, top face at floor_z) given in base_link and fixed in the planning frame when added
(the robot stands still in this session). Its top face is put MARGIN (1 cm) BELOW the floor: the feet stand on the real floor, and
a box at exactly floor height would put the start state in collision (MoveIt then refuses every plan). Runs until killed.
"""
import sys
import time

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene, PlanningSceneWorld
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Bool


def main():
    floor_z = float(sys.argv[1]) if len(sys.argv) > 1 else -0.107
    hz = float(sys.argv[2]) if len(sys.argv) > 2 else 2.0
    rclpy.init()
    n = rclpy.create_node("moveit_view_extras")
    cli = n.create_client(ApplyPlanningScene, "/apply_planning_scene")
    while not cli.wait_for_service(timeout_sec=2.0):
        n.get_logger().info("waiting for move_group /apply_planning_scene ...")
    co = CollisionObject()
    co.header.frame_id = "base_link"
    co.id = "floor"
    box = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[4.0, 4.0, 0.01])
    p = Pose()
    margin = 0.01
    p.position.z = floor_z - margin - 0.005
    p.orientation.w = 1.0
    co.primitives, co.primitive_poses, co.operation = [box], [p], CollisionObject.ADD
    f = cli.call_async(ApplyPlanningScene.Request(scene=PlanningScene(is_diff=True, world=PlanningSceneWorld(collision_objects=[co]))))
    rclpy.spin_until_future_complete(n, f, timeout_sec=10.0)
    ok = f.result() is not None and f.result().success
    print(f"floor added to the planning scene: {ok} (top face z {floor_z - margin:.3f} m in base_link = 1 cm below the floor)", flush=True)
    pub = n.create_publisher(Bool, "/wrist_camera/trigger", 2)
    try:
        while rclpy.ok():
            pub.publish(Bool(data=True))
            rclpy.spin_once(n, timeout_sec=0.0)
            time.sleep(1.0 / hz)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
