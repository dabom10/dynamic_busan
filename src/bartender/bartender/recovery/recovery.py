import time

class CentralDatabase:
    def __init__(self):
        self.order_table = {
            1: {"name": "김손님", "status": "완료", "keyword": "손님1"},
            2: {"name": "이영희", "status": "주문자 부재", "keyword": "영희 라떼"},
            3: {"name": "박철수", "status": "제조완료", "keyword": "철수 아아"}
        }

    def fetch_status(self, order_id):
        return self.order_table[order_id]["status"]

    def update_status(self, order_id, new_status):
        if order_id in self.order_table:
            old = self.order_table[order_id]["status"]
            self.order_table[order_id]["status"] = new_status
            # 기계적인 메시지 대신 간결하게 변경
            print(f"✅ [DB 업데이트] #{order_id}: {old} → {new_status}")

class RobotMissionControl:
    def __init__(self):
        self.db = CentralDatabase()
        self.shelf_location = "보관대_A"

    def run_robot_logic(self):
        print("🔍 주문 상태를 확인하는 중입니다...")
        
        current_status = self.db.fetch_status(2)
        
        if current_status == "주문자 부재":
            print(f"💡 2번 주문(이영희님)이 인식되지 않아 {self.shelf_location}로 옮깁니다.")
            time.sleep(1.0)
            
            self.db.update_status(2, "미 수령")
            
            print("\n🏃 다음 주문(3번) 처리를 시작합니다.")
            self._process_next_order(3)
            
            self._finalize_order_2(2)

    def _process_next_order(self, order_id):
        print(f"☕ {order_id}번 음료 제조 및 전달을 마쳤습니다.")
        self.db.update_status(order_id, "수령 완료")

    def _finalize_order_2(self, order_id):
        print(f"\n👀 {self.shelf_location}를 살피는 중... (수령 대기)")
        time.sleep(1.0)
        
        print(f"✨ 주문자가 음료를 가져갔습니다.")
        self.db.update_status(order_id, "수령 완료")

def run_robot_logic(args=None):
    controller = RobotMissionControl()
    controller.run_robot_logic()

if __name__ == "__main__":
    run_robot_logic()
