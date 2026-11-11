#!/usr/bin/env python3
import time
import requests
import logging
from typing import Optional, List, Dict, Any
import json

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

#这是最高层的实现，通过HTTP API提供控制接口，负责状态机的管理
class AutoStateMachine:
    def __init__(self, base_url: str = "http://127.0.0.1:5000"):  #5001 5002 ...
        """Initialize the automatic state machine controller
        
        Args:
            base_url: Base URL of the API server
        """
        self.base_url = base_url
        self.current_state = None
        self.navigation_points = []  # 添加导航点队列
        
    def get_current_state(self) -> str:
        """Get current state"""
        try:
            response = requests.get(f"{self.base_url}/status")
            if response.status_code == 200: #HTTP状态码200表示请求成功完成，服务器返回了预期的数据。
                self.current_state = response.json()["state"]
                return self.current_state
            else:
                logger.error(f"Failed to get status: {response.status_code}")
                return None
        except Exception as e:
            logger.error(f"An error occurred while getting the status: {str(e)}")
            return None
            
    def activate_idle(self) -> bool:
        """Activate idle state"""
        try:
            response = requests.post(f"{self.base_url}/activate_idle")
            if response.status_code == 200:
                self.current_state = response.json()["state"]
                logger.info(f"Activated idle state, current state: {self.current_state}")
                return True
            else:
                logger.error(f"Failed to activate idle state: {response.status_code}")
                return False
        except Exception as e:
            logger.error(f"An error occurred while activating the idle state: {str(e)}")
            return False
            
    def begin_takeoff(self) -> bool:
        """Start takeoff"""
        try:
            response = requests.post(f"{self.base_url}/begin_takeoff")
            if response.status_code == 200:
                self.current_state = response.json()["state"]
                logger.info(f"Takeoff started, current state: {self.current_state}")
                return True
            else:
                logger.error(f"Failed to start takeoff: {response.status_code}")
                return False
        except Exception as e:
            logger.error(f"An error occurred while starting takeoff: {str(e)}")
            return False
            
    def begin_landing(self) -> bool:
        """Start landing"""
        try:
            response = requests.post(f"{self.base_url}/begin_landing")
            if response.status_code == 200:
                self.current_state = response.json()["state"]
                logger.info(f"Landing started, current state: {self.current_state}")
                return True
            else:
                logger.error(f"Failed to start landing: {response.status_code}")
                return False
        except Exception as e:
            logger.error(f"An error occurred while starting landing: {str(e)}")
            return False
            
    def navigate_to(self, x: float, y: float, z: float) -> bool:
        """Navigate to specified position
        
        Args:
            x: x coordinate
            y: y coordinate
            z: z coordinate
            
        Returns:
            bool: Whether the navigation command was sent successfully
        """
        try:
            # 对于负数坐标，使用请求体而不是URL参数
            if x < 0 or y < 0 or z < 0:
                payload = {"x": x, "y": y, "z": z}
                response = requests.post(f"{self.base_url}/navigate", json=payload)
            else:
                response = requests.post(f"{self.base_url}/navigate/{x}/{y}/{z}")
                
            if response.status_code == 200:
                self.current_state = response.json()["state"]
                logger.info(f"Navigation command sent to position ({x}, {y}, {z}), current state: {self.current_state}")
                return True
            else:
                logger.error(f"Failed to send navigation command: {response.status_code}")
                return False
        except Exception as e:
            logger.error(f"An error occurred while sending navigation command: {str(e)}")
            return False
            
    def add_navigation_point(self, x: float, y: float, z: float) -> bool:
        """Add a single navigation point to the queue
        
        Args:
            x: x coordinate
            y: y coordinate
            z: z coordinate
            
        Returns:
            bool: Whether the point was added successfully
        """
        try:
            self.navigation_points.append({"x": x, "y": y, "z": z})
            return True
        except Exception as e:
            logger.error(f"Failed to add navigation point: {str(e)}")
            return False
            
    def add_navigation_points(self, points: List[Dict[str, float]]) -> bool:
        """Add multiple navigation points at once
        
        Args:
            points: List of dictionaries containing x, y, z coordinates
            
        Returns:
            bool: Whether all points were added successfully
        """
        try:
            # 先等待一段时间确保系统稳定
            time.sleep(0.5)  # 1 -> 0.5
            
            for point in points:
                # 使用navigate/append接口添加导航点
                x, y, z = point['x'], point['y'], point['z']
                
                # 对于负数坐标，使用请求体而不是URL参数
                if x < 0 or y < 0 or z < 0:
                    payload = {"x": x, "y": y, "z": z}
                    response = requests.post(f"{self.base_url}/navigate/append", json=payload)
                else:
                    response = requests.post(
                        f"{self.base_url}/navigate/append/{x}/{y}/{z}"
                    )
                    
                if response.status_code != 200:
                    logger.error(f"Failed to add navigation point: {point}")
                    return False
                logger.info(f"Added navigation point: ({x}, {y}, {z})")
                # 每个点添加后等待一小段时间
                time.sleep(0.2)
            return True
        except Exception as e:
            logger.error(f"An error occurred while adding navigation points: {str(e)}")
            return False
            
    def wait_for_state(self, target_state: str, timeout: float = 10.0) -> bool:
        """Wait for state machine to transition to specified state
        
        Args:
            target_state: Target state
            timeout: Timeout in seconds
            
        Returns:
            bool: Whether the target state was reached
        """
        start_time = time.time()
        while time.time() - start_time < timeout:
            current_state = self.get_current_state()
            if current_state == target_state:
                logger.info(f"Target state reached: {target_state}")
                return True
            time.sleep(0.1)
        logger.error(f"Waiting for state {target_state} timed out")
        return False
        
    def execute_sequence(self, sequence: List[Dict[str, Any]], batch_navigation: bool = False) -> bool:
        """Execute state machine transition sequence
        
        Args:
            sequence: Transition sequence, each element is a dictionary containing:
                - action: Action to execute (activate_idle/begin_takeoff/begin_landing/navigate)
                - wait_state: State to wait for
                - timeout: Wait timeout (optional)
                - wait_time: Optional additional wait time in this state
                - x/y/z: Navigation target coordinates (only for navigate action)
            batch_navigation: If True, send all navigation points at once. If False, send one point at a time.
                
        Returns:
            bool: Whether the sequence executed successfully
        """
        self.navigation_points = []  # 清空导航点队列
        for step in sequence:
            if step["action"] == "navigate":
                self.add_navigation_point(step["x"], step["y"], step["z"])

        nav_index = 0  # 导航点索引
        i = 0
        while i < len(sequence):
            step = sequence[i]
            action = step["action"]
            wait_state = step["wait_state"]
            timeout = step.get("timeout", 10.0)

            if action == "activate_idle":
                success = self.activate_idle()
                if not success or not self.wait_for_state(wait_state, timeout):
                    return False
            elif action == "begin_takeoff":
                success = self.begin_takeoff()
                if not success or not self.wait_for_state(wait_state, timeout):
                    return False
            elif action == "begin_landing":
                success = self.begin_landing()
                if not success or not self.wait_for_state(wait_state, timeout):
                    return False
            elif action == "navigate":
                if batch_navigation:
                    if nav_index == 0:
                        if len(self.navigation_points) > 1:
                            points_to_add = self.navigation_points[:-1]
                            if not self.add_navigation_points(points_to_add):
                                logger.error("Failed to add navigation points")
                                return False
                        if len(self.navigation_points) > 0:
                            last_point = self.navigation_points[-1]
                            success = self.navigate_to(last_point["x"], last_point["y"], last_point["z"])
                            if not success:
                                return False
                        # 批量模式下，导航后等待hovering
                        if not self.wait_for_state("hovering", timeout):
                            logger.error("Failed to reach hovering state after batch navigation")
                            return False
                        # 跳到所有navigate后的第一个动作
                        last_navigate_index = max(idx for idx, s in enumerate(sequence) if s["action"] == "navigate")
                        i = last_navigate_index + 1
                        nav_index += 1
                        continue  # 这里直接continue，主循环不会再i+=1
                    else:
                        # 其它navigate步骤直接跳过
                        i += 1
                        nav_index += 1
                        continue
                else:
                    if nav_index < len(self.navigation_points):
                        point = self.navigation_points[nav_index]
                        logger.info(f"Sending navigation point {nav_index+1}/{len(self.navigation_points)}: ({point['x']}, {point['y']}, {point['z']})")
                        current_state = self.get_current_state()
                        logger.info(f"Current state before navigation: {current_state}")
                        success = self.navigate_to(point["x"], point["y"], point["z"])
                        if not success:
                            return False
                        if not self.wait_for_state("hovering", timeout):
                            logger.error("Failed to reach hovering state after navigation")
                            return False
                        if "wait_time" in step:
                            logger.info(f"Waiting for additional {step['wait_time']} seconds")
                            time.sleep(step["wait_time"])
                            logger.info(f"Additional wait time completed")
                        nav_index += 1
                    else:
                        logger.error(f"Invalid navigation index: {nav_index}")
                        return False
            else:
                logger.error(f"Unknown action: {action}")
                return False
            if "wait_time" in step and action != "navigate":
                logger.info(f"Waiting for additional {step['wait_time']} seconds")
                time.sleep(step["wait_time"])
                logger.info(f"Additional wait time completed")
            i += 1
        return True

def control_multiple_drones():
    # 创建三个控制器实例
    drone1 = AutoStateMachine(base_url="http://127.0.0.1:5000")
    drone2 = AutoStateMachine(base_url="http://127.0.0.1:5001")
    drone3 = AutoStateMachine(base_url="http://127.0.0.1:5002")
    
    # 定义每架飞机的飞行序列
    sequence1 = [
        {
            "action": "activate_idle",
            "wait_state": "idle",
            "timeout": 5.0
        },
        {
            "action": "begin_takeoff",
            "wait_state": "hovering",
            "timeout": 10.0,
            "wait_time": 2.0
        },
        # 第一个导航点
        {
            "action": "navigate",
            "x": -1.0,
            "y": 0.0,
            "z": 1.1,
            "wait_state": "flying",
            "timeout": 60.0,
            "wait_time": 8.0  # Wait 2 seconds after reaching target
        },
        # 第二个导航点
        {
            "action": "navigate",
            "x": -1.0,
            "y": 1.0,
            "z": 1.1,
            "wait_state": "flying",
            "timeout": 60.0,
            "wait_time": 8.0
        },
        # # 第三个导航点
        # {
        #     "action": "navigate",
        #     "x": 0.5,
        #     "y": 0.0,
        #     "z": 0.6,
        #     "wait_state": "flying",
        #     "timeout": 60.0,
        #     "wait_time": 2.0
        # },
        # # 第四个导航点(返回起点)
        {
            "action": "navigate",
            "x": 0.0,
            "y": 0.0,
            "z": 1.1,
            "wait_state": "flying",
            "timeout": 60.0,
            "wait_time": 8.0
        },
        {
            "action": "begin_landing",
            "wait_state": "landing",
            "timeout": 10.0
        }
    ]
    
    sequence2 = [
        {
            "action": "activate_idle",
            "wait_state": "idle",
            "timeout": 5.0
        },
        {
            "action": "begin_takeoff",
            "wait_state": "hovering",
            "timeout": 10.0,
            "wait_time": 2.0
        },
        # 第一个导航点
        {
            "action": "navigate",
            "x": 0.0,
            "y": -0.8,
            "z": 0.6,
            "wait_state": "flying",
            "timeout": 60.0,
            "wait_time": 8.0  # Wait 2 seconds after reaching target
        },
        # 第二个导航点
        {
            "action": "navigate",
            "x": -0.8,
            "y": -0.8,
            "z": 0.6,
            "wait_state": "flying",
            "timeout": 60.0,
            "wait_time": 8.0
        },
        # # 第三个导航点
        # {
        #     "action": "navigate",
        #     "x": 0.5,
        #     "y": 0.0,
        #     "z": 0.6,
        #     "wait_state": "flying",
        #     "timeout": 60.0,
        #     "wait_time": 8.0
        # },
        # 第四个导航点(返回起点)
        {
            "action": "navigate",
            "x": 0.0,
            "y": 0.0,
            "z": 0.6,
            "wait_state": "flying",
            "timeout": 60.0,
            "wait_time": 8.0
        },
        {
            "action": "begin_landing",
            "wait_state": "landing",
            "timeout": 10.0
        }
    ]
    
    sequence3 = [
        {
            "action": "activate_idle",
            "wait_state": "idle",
            "timeout": 5.0
        },
        {
            "action": "begin_takeoff",
            "wait_state": "hovering",
            "timeout": 10.0,
            "wait_time": 2.0
        },
        # 第一个导航点
        {
            "action": "navigate",
            "x": 0.0,
            "y": 0.8,
            "z": 0.4,
            "wait_state": "flying",
            "timeout": 60.0,
            "wait_time": 8.0  # Wait 2 seconds after reaching target
        },
        # 第二个导航点
        {
            "action": "navigate",
            "x": -0.8,
            "y": -0.8,
            "z": 0.4,
            "wait_state": "flying",
            "timeout": 60.0,
            "wait_time": 8.0
        },
        # # 第三个导航点
        # {
        #     "action": "navigate",
        #     "x": 0.5,
        #     "y": 0.0,
        #     "z": 0.6,
        #     "wait_state": "flying",
        #     "timeout": 60.0,
        #     "wait_time": 8.0
        # },
        # 第四个导航点(返回起点)
        {
            "action": "navigate",
            "x": 0.0,
            "y": 0.0,
            "z": 0.4,
            "wait_state": "flying",
            "timeout": 60.0,
            "wait_time": 8.0
        },
        {
            "action": "begin_landing",
            "wait_state": "landing",
            "timeout": 10.0
        }
    ]
    
    # 同时执行三个序列
    import threading
    
    def run_sequence(drone, sequence):
        if drone.execute_sequence(sequence):
            logger.info(f"Drone sequence executed successfully")
        else:
            logger.error(f"Drone sequence execution failed")
    
    # 创建三个线程分别执行三个序列
    thread1 = threading.Thread(target=run_sequence, args=(drone1, sequence1))
    thread2 = threading.Thread(target=run_sequence, args=(drone2, sequence2))
    thread3 = threading.Thread(target=run_sequence, args=(drone3, sequence3))
    
    # 启动所有线程
    thread1.start()
    thread2.start()
    thread3.start()
    
    # 等待所有线程完成
    thread1.join()
    thread2.join()
    thread3.join()

# def main(): #这是单个无人机的实现，通过HTTP API提供控制接口，负责状态机的管理
#     # Create automatic state machine controller
#     auto_sm = AutoStateMachine()
    
#     # Define state machine transition sequence
#     sequence = [
#         {
#             "action": "activate_idle",
#             "wait_state": "idle",
#             "timeout": 5.0
#         },
#         {
#             "action": "begin_takeoff",
#             "wait_state": "hovering",
#             "timeout": 10.0,
#             "wait_time": 1.0  # Hover for 1 seconds
#         },
#         # 第一个导航点
#         {
#             "action": "navigate",
#             "x": 0.0,
#             "y": 0.5,
#             "z": 0.6,
#             "wait_state": "flying",
#             "timeout": 60.0,
#             "wait_time": 2.0  # Wait 2 seconds after reaching target
#         },
#         # 第二个导航点
#         {
#             "action": "navigate",
#             "x": 0.5,
#             "y": 0.5,
#             "z": 0.6,
#             "wait_state": "flying",
#             "timeout": 60.0,
#             "wait_time": 2.0
#         },
#         # 第三个导航点
#         {
#             "action": "navigate",
#             "x": 0.5,
#             "y": 0.0,
#             "z": 0.6,
#             "wait_state": "flying",
#             "timeout": 60.0,
#             "wait_time": 2.0
#         },
#         # 第四个导航点(返回起点)
#         {
#             "action": "navigate",
#             "x": 0.0,
#             "y": 0.0,
#             "z": 0.6,
#             "wait_state": "flying",
#             "timeout": 60.0,
#             "wait_time": 2.0
#         },
#         # 第一个导航点
#         {
#             "action": "navigate",
#             "x": 0.0,
#             "y": 0.5,
#             "z": 0.8,
#             "wait_state": "flying",
#             "timeout": 60.0,
#             "wait_time": 2.0  # Wait 2 seconds after reaching target
#         },
#         # 第二个导航点
#         {
#             "action": "navigate",
#             "x": 0.5,
#             "y": 0.5,
#             "z": 0.8,
#             "wait_state": "flying",
#             "timeout": 60.0,
#             "wait_time": 2.0
#         },
#         # 第三个导航点
#         {
#             "action": "navigate",
#             "x": 0.5,
#             "y": 0.0,
#             "z": 0.8,
#             "wait_state": "flying",
#             "timeout": 60.0,
#             "wait_time": 2.0
#         },
#         # 第四个导航点(返回起点)
#         {
#             "action": "navigate",
#             "x": 0.0,
#             "y": 0.0,
#             "z": 0.8,
#             "wait_state": "flying",
#             "timeout": 60.0,
#             "wait_time": 2.0
#         },
#                 # 第一个导航点
#         {
#             "action": "navigate",
#             "x": 0.0,
#             "y": 0.5,
#             "z": 0.4,
#             "wait_state": "flying",
#             "timeout": 60.0,
#             "wait_time": 2.0  # Wait 2 seconds after reaching target
#         },
#         # 第二个导航点
#         {
#             "action": "navigate",
#             "x": 0.5,
#             "y": 0.5,
#             "z": 0.4,
#             "wait_state": "flying",
#             "timeout": 60.0,
#             "wait_time": 2.0
#         },
#         # 第三个导航点
#         {
#             "action": "navigate",
#             "x": 0.5,
#             "y": 0.0,
#             "z": 0.4,
#             "wait_state": "flying",
#             "timeout": 60.0,
#             "wait_time": 2.0
#         },
#         # 第四个导航点(返回起点)
#         {
#             "action": "navigate",
#             "x": 0.0,
#             "y": 0.0,
#             "z": 0.4,
#             "wait_state": "flying",
#             "timeout": 60.0,
#             "wait_time": 2.0
#         },
#         {
#             "action": "begin_landing",
#             "wait_state": "landing",
#             "timeout": 10.0
#         }
#     ]
    
#     # Execute sequence
#     if auto_sm.execute_sequence(sequence): #如果execute_sequence最后返回True
#         logger.info("State machine sequence executed successfully")
#     else:#如果execute_sequence最后返回False
#         logger.error("State machine sequence execution failed")

if __name__ == "__main__":
    #main() 
    control_multiple_drones()