# return 角度一律 radians
import math
from rclpy.node import Node
from pros_car_py.car_models import DeviceDataTypeEnum
import numpy as np
import time
import sys
from scipy.spatial.transform import Rotation as R
import pybullet as p
import numpy as np
import threading


class ArmController:
    def __init__(self, ros_communicator, data_processor):
        self.ros_communicator = ros_communicator
        self.data_processor = data_processor
        
        # ==========================================
        # 1. 手臂基礎設定 (統一管理區)
        # ==========================================
        self.base_link_name = 'arm_base_link'
        
        # 完美移植你的設定：動態定義所有關節 (包含手臂與夾爪)
        # 只要在這裡增減數量，下方的控制與發布邏輯會自動適應！
        self.joint_limits = [
            {"min_angle": 0,  "max_angle": 180, "init": 180},  # Joint 0
            {"min_angle": 0,  "max_angle": 240, "init": 0},  # Joint 1 
            {"min_angle": 0,  "max_angle": 90, "init": 90},  # Joint 2 
        ]
        
        # 根據上面的設定，自動產生當前角度的陣列
        self.joint_angles = [joint["init"] for joint in self.joint_limits]
        
        # 控制參數 (對應你原本的 delta_angle = 10)
        self.manual_step = 10.0   
        
        print(f"🦾 Arm Controller Initialized: {len(self.joint_limits)} Joints Managed.")

    # ==========================================
    # 2. 手動控制邏輯 (Manual Control)
    # ==========================================
    def manual_control(self, index, key):
        """處理手動按鍵輸入，並根據 index 控制特定關節"""
        
        # 處理不依賴 index 的全域指令 ('b', 'q')
        if key == "b":  
            # 🌟 重置手臂：讀取 __init__ 裡面設定的 init 初始角度
            self.joint_angles = [joint["init"] for joint in self.joint_limits]
            self._clamp_and_publish()
            print("手臂已重置為初始角度。")
            return False
            
        elif key == "q":  
            # 結束控制
            print("結束手臂手動控制。")
            return True

        # 處理針對特定 index 的控制 ('i', 'k')
        if 0 <= index < len(self.joint_limits):
            if key == "i":
                self.joint_angles[index] += self.manual_step
            elif key == "k":
                self.joint_angles[index] -= self.manual_step
            else:
                print(f"按鍵 '{key}' 無效，請使用 'i', 'k', 'b', 或 'q'。")
                return False
                
            # 計算完畢後，進行安全檢查並發布
            self._clamp_and_publish()
        else:
            print(f"索引 {index} 無效，請確保其在範圍內（0-{len(self.joint_limits) - 1}）。")
            
        return False

    # ==========================================
    # 3. 狀態限制與發布
    # ==========================================
    def _clamp_and_publish(self):
        """確保所有數值在安全範圍內，並轉換為「弧度」後發布"""
        
        # 1. 限制各關節角度在 min 與 max 之間 (維持角度計算)
        for i in range(len(self.joint_limits)):
            min_a = self.joint_limits[i]["min_angle"]
            max_a = self.joint_limits[i]["max_angle"]
            self.joint_angles[i] = max(min_a, min(max_a, self.joint_angles[i]))
            
        # 2. 🌟 將所有的「角度 (Degrees)」轉換為「弧度 (Radians)」
        joint_pos_radians = [math.radians(float(a)) for a in self.joint_angles]
        
        # 3. 透過 ros_communicator 發布 (傳送弧度陣列)
        self.ros_communicator.publish_robot_arm_angle(joint_pos_radians)

        # 4. 在終端機同時印出兩種單位，方便你監看除錯！
        degrees_str = [round(a, 1) for a in self.joint_angles]
        radians_str = [round(a, 3) for a in joint_pos_radians]
        print(f"📐 內部角度: {degrees_str}  ->  發布弧度: {radians_str}")
    
    def auto_control(self, key=None, mode="auto_arm_control"):
        pass