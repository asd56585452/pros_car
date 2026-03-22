from pros_car_py.nav2_utils import (
    get_yaw_from_quaternion,
    get_direction_vector,
    get_angle_to_target,
    calculate_angle_point,
    cal_distance,
)
import math
import time


class Nav2Processing:
    def __init__(self, ros_communicator, data_processor):
        self.ros_communicator = ros_communicator
        self.data_processor = data_processor
        self.finishFlag = False
        self.global_plan_msg = None
        self.index = 0
        self.index_length = 0
        self.recordFlag = 0
        self.goal_published_flag = False

    def reset_nav_process(self):
        self.finishFlag = False
        self.recordFlag = 0
        self.goal_published_flag = False
        self.stuck_duration = 0.0
        self.backing_up_end_time = 0.0
        self.last_position = None
        self.last_yaw = None

    def finish_nav_process(self):
        self.finishFlag = True
        self.recordFlag = 1

    def get_finish_flag(self):
        return self.finishFlag

    def get_action_from_nav2_plan(self, goal_coordinates=None,target_distance_threshold=0.1,diff_angle_threshold=5.0):
        # 1. 發布新目標
        if goal_coordinates is not None and not self.goal_published_flag:
            self.ros_communicator.publish_goal_pose(goal_coordinates)
            self.goal_published_flag = True

        # 2. 取得車體與路徑資料
        car_pose = self.data_processor.get_processed_amcl_pose()
        goal_position = self.ros_communicator.get_latest_goal()
        plan_data = self.data_processor.get_processed_received_global_plan()

        if not car_pose or not goal_position or not plan_data:
            return "STOP"

        car_position, car_orientation = car_pose
        orientation_points, coordinates = plan_data

        if not coordinates or not orientation_points:
            return "STOP"

        car_yaw = math.degrees(2.0 * math.atan2(car_orientation[2], car_orientation[3]))
        target_distance = cal_distance(car_position, goal_position)

       # ==========================================
        # 階段 1：判定是否已經完美抵達 (避免抵達時誤判為卡住)
        # ==========================================
        is_arrived = False
        goal_yaw = None
        diff_angle_for_align = 0.0

        if target_distance < target_distance_threshold:
            # 嘗試取得目標姿態 (Yaw)
            if goal_coordinates is not None and len(goal_coordinates) >= 3:
                goal_yaw = goal_coordinates[2]
            else:
                try:
                    goal_pose_msg = self.ros_communicator.get_goal_pose() 
                    goal_z = goal_pose_msg.orientation.z
                    goal_w = goal_pose_msg.orientation.w
                    goal_yaw = math.degrees(2.0 * math.atan2(goal_z, goal_w))
                except Exception as e:
                    print(f"無法取得目標方向，略過旋轉: {e}")

            # 檢查角度是否對齊
            if goal_yaw is not None:
                diff_angle_for_align = goal_yaw - car_yaw
                while diff_angle_for_align > 180: diff_angle_for_align -= 360
                while diff_angle_for_align < -180: diff_angle_for_align += 360

                if abs(diff_angle_for_align) < diff_angle_threshold:
                    is_arrived = True
            else:
                # 若沒有指定目標角度，距離小於 0.5m 就算抵達
                is_arrived = True

        if is_arrived:
            self.finishFlag = True
            self.ros_communicator.reset_nav2() 
            self.stuck_duration = 0.0 # 清除卡住計時
            return "STOP"


        # ==========================================
        # 階段 2：🌟 後退脫困機制 (加入旋轉判定)
        # ==========================================
        current_time = time.time()
        
        # 初始化卡住偵測變數
        if not hasattr(self, 'backing_up_end_time'):
            self.backing_up_end_time = 0.0
            self.stuck_duration = 0.0
            self.last_check_time = current_time
            self.last_position = car_position
            self.last_yaw = car_yaw

        # 如果正在強制後退中，直接維持後退動作
        if current_time < self.backing_up_end_time:
            return "BACKWARD"
        elif current_time < self.backing_up_end_time + 3.0:
            return "STOP"

        # 每 1 秒檢查一次狀態
        if current_time - self.last_check_time > 1.0:
            if self.last_position is not None and getattr(self, 'last_yaw', None) is not None:
                dist_moved = cal_distance(self.last_position, car_position)
                
                # 計算這 1 秒內的角度變化量 (取絕對值)
                yaw_diff = car_yaw - self.last_yaw
                while yaw_diff > 180: yaw_diff -= 360
                while yaw_diff < -180: yaw_diff += 360
                yaw_changed = abs(yaw_diff)
                
                # 💡 終極卡住判定：
                # 既然程式執行到這 (is_arrived == False)，代表車子「必須」要移動或旋轉。
                # 如果它位移不到 3 公分「而且」旋轉不到 3 度，才代表它真的卡死了！
                if dist_moved < 0.03 and yaw_changed < 3.0:
                    self.stuck_duration += 1.0
                else:
                    # 只要有在正常前進，或是正常原地旋轉，就解除卡住警報
                    self.stuck_duration = 0.0
            
            self.last_position = car_position
            self.last_yaw = car_yaw
            self.last_check_time = current_time

        # 連續 3 秒卡死，觸發 1.5 秒的後退
        if self.stuck_duration >= 3.0:
            print("[Nav2Processing] ⚠️ 偵測到車體卡住 (無位移且無旋轉)！啟動後退脫困...")
            self.backing_up_end_time = current_time + 3.0
            self.stuck_duration = 0.0
            return "BACKWARD"


        # ==========================================
        # 階段 3：決定一般導航動作
        # ==========================================
        if target_distance < target_distance_threshold:
            # 距離夠近，但尚未抵達 (因為上面 is_arrived == False)，開始原地旋轉對齊
            if diff_angle_for_align > 0:
                return "COUNTERCLOCKWISE_ROTATION_SLOW"
            else:
                return "CLOCKWISE_ROTATION_SLOW"
        else:
            # 距離大於 0.5m，尋找路線前方目標點
            target_x, target_y = None, None
            for i in range(self.index, len(coordinates)):
                tx, ty = coordinates[i]
                if cal_distance(car_position, [tx, ty]) >= target_distance_threshold:
                    target_x, target_y = tx, ty
                    self.index = i 
                    break

            if target_x is None:
                target_x, target_y = coordinates[-1]

            target_pos = [target_x, target_y]
            diff_angle = calculate_angle_point(
                car_orientation[2], car_orientation[3], car_position[:2], target_pos
            )

            # 角度在正負 diff_angle_threshold 度內直走，否則原地轉向對準路線
            if diff_angle < diff_angle_threshold and diff_angle > -diff_angle_threshold:
                return "FORWARD"
            elif diff_angle <= -diff_angle_threshold:
                return "CLOCKWISE_ROTATION"
            elif diff_angle >= diff_angle_threshold:
                return "COUNTERCLOCKWISE_ROTATION"

        return "STOP"

    def get_action_from_nav2_plan_no_dynamic_p_2_p(self, goal_coordinates=None):
        if goal_coordinates is not None and not self.goal_published_flag:
            self.ros_communicator.publish_goal_pose(goal_coordinates)
            self.goal_published_flag = True

        # 只抓第一次路径
        if self.recordFlag == 0:
            if not self.check_data_availability():
                return "STOP"
            else:
                print("Get first path")
                self.index = 0
                self.global_plan_msg = (
                    self.data_processor.get_processed_received_global_plan_no_dynamic()
                )
                self.recordFlag = 1
                action_key = "STOP"

        car_position, car_orientation = self.data_processor.get_processed_amcl_pose()

        goal_position = self.ros_communicator.get_latest_goal()
        target_distance = cal_distance(car_position, goal_position)

        # 抓最近的物標(可調距離)
        target_x, target_y = self.get_next_target_point(car_position)

        if target_x is None or target_distance < 0.5:
            self.ros_communicator.reset_nav2()
            self.finish_nav_process()
            return "STOP"

        # 計算角度誤差
        diff_angle = self.calculate_diff_angle(
            car_position, car_orientation, target_x, target_y
        )
        if diff_angle < 20 and diff_angle > -20:
            action_key = "FORWARD"
        elif diff_angle < -20 and diff_angle > -180:
            action_key = "CLOCKWISE_ROTATION"
        elif diff_angle > 20 and diff_angle < 180:
            action_key = "COUNTERCLOCKWISE_ROTATION"
        return action_key

    def check_data_availability(self):
        return (
            self.data_processor.get_processed_received_global_plan_no_dynamic()
            and self.data_processor.get_processed_amcl_pose()
            and self.ros_communicator.get_latest_goal()
        )

    def get_next_target_point(self, car_position, min_required_distance=0.5):
        """
        選擇距離車輛 min_required_distance 以上最短路徑然後返回 target_x, target_y
        """
        if self.global_plan_msg is None or self.global_plan_msg.poses is None:
            print("Error: global_plan_msg is None or poses is missing!")
            return None, None
        while self.index < len(self.global_plan_msg.poses) - 1:
            target_x = self.global_plan_msg.poses[self.index].pose.position.x
            target_y = self.global_plan_msg.poses[self.index].pose.position.y
            distance_to_target = cal_distance(car_position, (target_x, target_y))

            if distance_to_target < min_required_distance:
                self.index += 1
            else:
                self.ros_communicator.publish_selected_target_marker(
                    x=target_x, y=target_y
                )
                return target_x, target_y

        return None, None

    def calculate_diff_angle(self, car_position, car_orientation, target_x, target_y):
        target_pos = [target_x, target_y]
        diff_angle = calculate_angle_point(
            car_orientation[2], car_orientation[3], car_position[:2], target_pos
        )
        return diff_angle

    def filter_negative_one(self, depth_list):
        return [depth for depth in depth_list if depth != -1.0]
    
    def get_wheel_speeds_from_cmd_vel(self, track_width=0.2875, wheel_radius=0.04940625, unit='deg_s'):
        """
        將 /cmd_vel 的線速度與角速度，轉換為四輪轉速
        :param track_width: 輪距 (公尺)
        :param wheel_radius: 輪胎半徑 (公尺)
        :param unit: 輸出單位 ('m_s' 線速度, 'rad_s' 弧度角速度, 'deg_s' 角度角速度)
        """
        linear_x, angular_z = self.data_processor.get_processed_cmd_vel()
        
        # 如果 Nav2 沒有下達速度指令 (或者還沒啟動)
        if linear_x == 0.0 and angular_z == 0.0:
            return [0.0, 0.0, 0.0, 0.0]
            
        # 1. 先計算出左右輪胎的「切線線速度」 (m/s)
        left_linear_speed = linear_x - (angular_z * track_width / 2.0)
        right_linear_speed = linear_x + (angular_z * track_width / 2.0)
        
        # 2. 依照指定的單位進行轉換
        if unit == 'm_s':
            # 保持公尺/秒
            left_speed = left_linear_speed
            right_speed = right_linear_speed
            
        elif unit == 'rad_s':
            # 轉換為弧度/秒 (rad/s) = v / r
            left_speed = left_linear_speed / wheel_radius
            right_speed = right_linear_speed / wheel_radius
            
        elif unit == 'deg_s':
            # 轉換為角度/秒 (deg/s) = (v / r) * (180 / pi)
            left_speed = (left_linear_speed / wheel_radius) * (180.0 / math.pi)
            right_speed = (right_linear_speed / wheel_radius) * (180.0 / math.pi)
            
        else:
            left_speed = left_linear_speed
            right_speed = right_linear_speed
        
        # 3. 組裝成 4 輪陣列: [左後, 右後, 左前, 右前]
        wheel_velocities = [
            left_speed,   # rear_left
            right_speed,  # rear_right
            left_speed,   # front_left
            right_speed   # front_right
        ]
        
        return wheel_velocities

    def camera_nav(self):
        """
        YOLO 目標資訊 (yolo_target_info) 說明：

        - 索引 0 (index 0)：
            - 表示是否成功偵測到目標
            - 0：未偵測到目標
            - 1：成功偵測到目標

        - 索引 1 (index 1)：
            - 目標的深度距離 (與相機的距離，單位為公尺)，如果沒偵測到目標就回傳 0
            - 與目標過近時(大約 40 公分以內)會回傳 -1

        - 索引 2 (index 2)：
            - 目標相對於畫面正中心的像素偏移量
            - 若目標位於畫面中心右側，數值為正
            - 若目標位於畫面中心左側，數值為負
            - 若沒有目標則回傳 0

        畫面 n 個等分點深度 (camera_multi_depth) 說明 :

        - 儲存相機畫面中央高度上 n 個等距水平點的深度值。
        - 若距離過遠、過近（小於 40 公分）或是實體相機有時候深度會出一些問題，則該點的深度值將設定為 -1。
        """
        yolo_target_info = self.data_processor.get_yolo_target_info()
        camera_multi_depth = self.data_processor.get_camera_x_multi_depth()
        if camera_multi_depth == None or yolo_target_info == None:
            return "STOP"

        camera_forward_depth = self.filter_negative_one(camera_multi_depth[7:13])
        camera_left_depth = self.filter_negative_one(camera_multi_depth[0:7])
        camera_right_depth = self.filter_negative_one(camera_multi_depth[13:20])

        action = "STOP"
        limit_distance = 0.7

        if all(depth > limit_distance for depth in camera_forward_depth):
            if yolo_target_info[0] == 1:
                if yolo_target_info[2] > 200.0:
                    action = "CLOCKWISE_ROTATION_SLOW"
                elif yolo_target_info[2] < -200.0:
                    action = "COUNTERCLOCKWISE_ROTATION_SLOW"
                else:
                    if yolo_target_info[1] < 0.8:
                        action = "STOP"
                    else:
                        action = "FORWARD_SLOW"
            else:
                action = "FORWARD"
        elif any(depth < limit_distance for depth in camera_left_depth):
            action = "CLOCKWISE_ROTATION"
        elif any(depth < limit_distance for depth in camera_right_depth):
            action = "COUNTERCLOCKWISE_ROTATION"
        return action

    def camera_nav_unity(self):
        """
        YOLO 目標資訊 (yolo_target_info) 說明：

        - 索引 0 (index 0)：
            - 表示是否成功偵測到目標
            - 0：未偵測到目標
            - 1：成功偵測到目標

        - 索引 1 (index 1)：
            - 目標的深度距離 (與相機的距離，單位為公尺)，如果沒偵測到目標就回傳 0
            - 與目標過近時(大約 40 公分以內)會回傳 -1

        - 索引 2 (index 2)：
            - 目標相對於畫面正中心的像素偏移量
            - 若目標位於畫面中心右側，數值為正
            - 若目標位於畫面中心左側，數值為負
            - 若沒有目標則回傳 0

        畫面 n 個等分點深度 (camera_multi_depth) 說明 :

        - 儲存相機畫面中央高度上 n 個等距水平點的深度值。
        - 若距離過遠、過近（小於 40 公分）或是實體相機有時候深度會出一些問題，則該點的深度值將設定為 -1。
        """
        yolo_target_info = self.data_processor.get_yolo_target_info()
        camera_multi_depth = self.data_processor.get_camera_x_multi_depth()
        yolo_target_info[1] *= 100.0
        camera_multi_depth = list(
            map(lambda x: x * 100.0, self.data_processor.get_camera_x_multi_depth())
        )

        if camera_multi_depth == None or yolo_target_info == None:
            return "STOP"

        camera_forward_depth = self.filter_negative_one(camera_multi_depth[7:13])
        camera_left_depth = self.filter_negative_one(camera_multi_depth[0:7])
        camera_right_depth = self.filter_negative_one(camera_multi_depth[13:20])
        action = "STOP"
        limit_distance = 10.0
        print(yolo_target_info[1])
        if all(depth > limit_distance for depth in camera_forward_depth):
            if yolo_target_info[0] == 1:
                if yolo_target_info[2] > 200.0:
                    action = "CLOCKWISE_ROTATION_SLOW"
                elif yolo_target_info[2] < -200.0:
                    action = "COUNTERCLOCKWISE_ROTATION_SLOW"
                else:
                    if yolo_target_info[1] < 2.0:
                        action = "STOP"
                    else:
                        action = "FORWARD_SLOW"
            else:
                action = "FORWARD"
        elif any(depth < limit_distance for depth in camera_left_depth):
            action = "CLOCKWISE_ROTATION"
        elif any(depth < limit_distance for depth in camera_right_depth):
            action = "COUNTERCLOCKWISE_ROTATION"
        return action

    def stop_nav(self):
        return "STOP"
