import mujoco
import mujoco.viewer
import time
import json
import os
import numpy as np
from datetime import datetime

# ========== 模型定义（修复旋转问题）==========
XML = """
<mujoco>
<option gravity="0 0 0" timestep="0.001"/>
<asset>
    <mesh file="C:/GCPD/robotic+arm+3d+model/tripo_convert_dab5fc6a-b1cf-45e0-8a56-bee33dc45877.obj" name="brush_mesh" scale="0.1 0.1 0.1"/>
    <mesh file="C:/GCPD/back_model.obj/tripo_convert_ee38a00f-f8ba-4f24-9d1c-ad455530561d.obj" name="back_mesh" scale="0.1 0.1 0.1"/>
</asset>
<worldbody>
    <body name="back" pos="0.2 0 0.1">
        <geom type="mesh" mesh="back_mesh" rgba="0.8 0.7 0.6 1" mass="10"/>
    </body>
    
    <!-- 固定基座 -->
    <body name="base" pos="0 0 0.12">
        <geom type="box" size="0.02 0.02 0.02" rgba="0.5 0.5 0.5 1" mass="1"/>
        
        <!-- X 轴滑动副 -->
        <body name="slider_x" pos="0 0 0">
            <joint type="slide" axis="1 0 0" range="-0.2 0.2" damping="0.1"/>
            
            <!-- Y 轴滑动副 -->
            <body name="slider_y" pos="0 0 0">
                <joint type="slide" axis="0 1 0" range="-0.15 0.15" damping="0.1"/>
                
                <!-- 机械臂（固定高度，不允许旋转） -->
                <body name="scrubber" pos="0 0 0">
                    <geom type="mesh" mesh="brush_mesh" euler="0 90 0" rgba="1 0.8 0.8 1" mass="0.2"/>
                    <site name="force_sensor" pos="0 0 0" size="0.01" type="sphere" rgba="0 1 0 1" group="3"/>
                </body>
            </body>
        </body>
    </body>
</worldbody>

<sensor>
    <force name="scrubber_force" site="force_sensor"/>
</sensor>

<!-- 执行器（用于主动控制） -->
<actuator>
    <motor name="motor_x" joint="slider_x" gear="1" ctrlrange="-1 1"/>
    <motor name="motor_y" joint="slider_y" gear="1" ctrlrange="-1 1"/>
</actuator>
</mujoco>
"""

print("正在加载模型...")
model = mujoco.MjModel.from_xml_string(XML)
data = mujoco.MjData(model)

# 获取索引
force_sensor_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, "scrubber_force")
base_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base")

# 获取关节索引
slider_x_jnt = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "slider_x")
slider_y_jnt = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "slider_y")

# 获取执行器索引
motor_x_act = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "motor_x")
motor_y_act = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "motor_y")

# 自由度索引
qpos_x = model.jnt_qposadr[slider_x_jnt]
qpos_y = model.jnt_qposadr[slider_y_jnt]

# ========== 关键参数 ==========
COORD_FILE = 'C:/GCPD/face_coord.json'

# 校准参数
CALIB_OFFSET_X = 0.0
CALIB_OFFSET_Y = 0.0
SIM_RANGE_X = 0.4
SIM_RANGE_Y = 0.3
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480

# 滤波参数
ALPHA = 0.3
DEAD_ZONE = 0.005

# 超时参数
TIMEOUT_MS = 500

# 状态变量
last_sim_x = 0.0
last_sim_y = 0.0
face_lost_count = 0

# 控制参数
KP = 10.0  # 比例增益（PD 控制）
KD = 1.0   # 微分增益

print("=" * 50)
print("搓背机器人仿真系统 - 修复版（无旋转）")
print("=" * 50)
print(f"关节范围：X={model.jnt_range[slider_x_jnt]}, Y={model.jnt_range[slider_y_jnt]}")
print(f"滤波系数：{ALPHA}, 死区：{DEAD_ZONE}")
print(f"PD 控制：Kp={KP}, Kd={KD}")
print("=" * 50)
print("仿真端已启动... 等待视觉坐标...")
print("按 'q' 退出仿真窗口")
print("=" * 50)

def map_pixel_to_sim(px, py):
    """像素坐标 → 仿真世界坐标"""
    norm_x = px / CAMERA_WIDTH
    norm_y = py / CAMERA_HEIGHT
    sim_x = (norm_x - 0.5) * SIM_RANGE_X + CALIB_OFFSET_X
    sim_y = (norm_y - 0.5) * SIM_RANGE_Y + CALIB_OFFSET_Y
    return sim_x, sim_y

def lowpass_filter(current, target, alpha):
    """一阶低通滤波"""
    return current + alpha * (target - current)

def apply_dead_zone(current, target, dead_zone):
    """应用死区阈值"""
    diff = target - current
    if abs(diff) < dead_zone:
        return current
    return target

# 初始化位置
data.qpos[qpos_x] = last_sim_x
data.qpos[qpos_y] = last_sim_y

with mujoco.viewer.launch_passive(model, data) as viewer:
    print("✅ 仿真窗口已打开")
    
    while viewer.is_running():
        current_time = datetime.now()
        target_x, target_y = None, None
        
        # 读取视觉坐标
        if os.path.exists(COORD_FILE):
            try:
                with open(COORD_FILE, 'r') as f:
                    coord = json.load(f)
                
                if 'timestamp' in coord:
                    coord_time = datetime.fromisoformat(coord['timestamp'])
                    age_ms = (current_time - coord_time).total_seconds() * 1000
                    
                    if age_ms > TIMEOUT_MS:
                        face_lost_count += 1
                        if face_lost_count % 100 == 0:
                            print(f"⚠️ 坐标超时 ({age_ms:.0f}ms)")
                    else:
                        px = coord.get('x')
                        py = coord.get('y')
                        if px is not None and py is not None:
                            target_x, target_y = map_pixel_to_sim(px, py)
                            face_lost_count = 0
                else:
                    px = coord.get('x')
                    py = coord.get('y')
                    if px is not None and py is not None:
                        target_x, target_y = map_pixel_to_sim(px, py)
                        face_lost_count = 0
                        
            except Exception as e:
                face_lost_count += 1
                if face_lost_count % 100 == 0:
                    print(f"📁 读取出错：{e}")
        else:
            face_lost_count += 1
            if face_lost_count % 100 == 0:
                print("⚠️ 坐标文件不存在")
        
        # 处理坐标更新
        if target_x is not None and target_y is not None:
            new_x = apply_dead_zone(last_sim_x, target_x, DEAD_ZONE)
            new_y = apply_dead_zone(last_sim_y, target_y, DEAD_ZONE)
            
            last_sim_x = lowpass_filter(last_sim_x, new_x, ALPHA)
            last_sim_y = lowpass_filter(last_sim_y, new_y, ALPHA)
        
        # ========== PD 控制 ==========
        # 计算位置误差
        error_x = last_sim_x - data.qpos[qpos_x]
        error_y = last_sim_y - data.qpos[qpos_y]
        
        # 计算速度（微分项）
        vel_x = data.qvel[qpos_x]
        vel_y = data.qvel[qpos_y]
        
        # PD 控制输出
        ctrl_x = KP * error_x - KD * vel_x
        ctrl_y = KP * error_y - KD * vel_y
        
        # 限制控制量
        ctrl_x = np.clip(ctrl_x, -1, 1)
        ctrl_y = np.clip(ctrl_y, -1, 1)
        
        # 应用控制
        data.ctrl[motor_x_act] = ctrl_x
        data.ctrl[motor_y_act] = ctrl_y
        
        # 物理步进
        mujoco.mj_step(model, data)
        
        # 力传感器读数
        force = data.sensordata[force_sensor_id*3 : force_sensor_id*3+3]
        if abs(force[2]) > 0.01:
            print(f"💪 接触力：{force[2]:.3f} N")
        
        viewer.sync()
        time.sleep(0.01)

print("\n✅ 仿真已退出")
