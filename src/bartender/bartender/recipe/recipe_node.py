#!/usr/bin/env python3
"""
Recipe Node - Action Server for recipe motion control
"""
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle

from bartender_interfaces.action import Motion


class RecipeController(Node):
    def __init__(self):
        super().__init__("recipe_node")
        self.get_logger().info("Recipe Node initialized")

        # Action Server 생성
        self._action_server = ActionServer(
            self,
            Motion,
            'recipe/motion',  # Action 이름
            self.execute_callback
        )

        self.get_logger().info("Recipe Action Server ready (recipe/motion)")

    def execute_callback(self, goal_handle: ServerGoalHandle):
        """Action 실행 콜백"""
        self.get_logger().info(f"Received goal: motion_id={goal_handle.request.motion_id}, "
                               f"name={goal_handle.request.motion_name}, "
                               f"duration={goal_handle.request.duration_ms}ms")

        motion_name = goal_handle.request.motion_name
        duration_ms = goal_handle.request.duration_ms

        # Feedback 메시지
        feedback_msg = Motion.Feedback()

        # 모션 실행 (시뮬레이션)
        start_time = time.time()
        total_steps = 10

        for i in range(total_steps):
            progress = int((i + 1) / total_steps * 100)

            feedback_msg.progress = progress
            feedback_msg.current_step = f"Recipe {motion_name} step {i + 1}/{total_steps}"
            goal_handle.publish_feedback(feedback_msg)

            self.get_logger().info(f"Progress: {progress}% - {feedback_msg.current_step}")

            time.sleep(duration_ms / 1000 / total_steps)

        elapsed_ms = int((time.time() - start_time) * 1000)

        goal_handle.succeed()

        result = Motion.Result()
        result.success = True
        result.message = f"Recipe '{motion_name}' completed successfully"
        result.total_time_ms = elapsed_ms

        self.get_logger().info(f"Recipe completed in {elapsed_ms}ms")

        return result


def main(args=None):
    rclpy.init(args=args)
    node = RecipeController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
