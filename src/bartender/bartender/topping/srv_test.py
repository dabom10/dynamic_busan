import rclpy
from rclpy.node import Node
from srv_interfaces.srv import DrinkDelivery
import DR_init

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"
ROBOT_TCP = "GripperDA_v2"

class Delivery(Node):
    def __init__(self):
        super().__init__("topping", namespace=ROBOT_ID)

        # 서비스 클라이언트 생성
        self.cli = self.create_client(DrinkDelivery,'get_pose')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("서비스 대기 중...")
        self.req = DrinkDelivery.Request()
    
    def send_request(self):
        from DSR_ROBOT2 import movel, posx,movej
        self.req.finish=True
        future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)

        response = future.result()
        self.get_logger().info(f"받은 좌표:{response.goal_position}")
        pos = list(response.goal_position)
        
        t = type(response.goal_position)
        self.get_logger().info(f"타입:{t}")

        if len(pos) <= 0 :
            print("반환된 좌표가 없습니다")
            return

        self.get_logger().info("홈위치로 이동")
        movej([0,0,90,0,90,0],vel=60,acc=60)
        
        self.get_logger().info(f"손님 위치로 이동")
        movel(posx(pos), vel=60,acc=60)
        # print("pos 확인",list(pos))

        self.get_logger().info(f"이동 완료")

    


def main():
    rclpy.init()
    DR_init.__dsr__id = ROBOT_ID
    DR_init.__dsr__model = ROBOT_MODEL

    node = Delivery()
    DR_init.__dsr__node = node

    try:
        # from DSR_ROBOT2 import movel, posx
        node.send_request()
        # rclpy.spin(node)

        

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()



