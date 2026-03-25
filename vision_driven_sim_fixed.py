import mujoco
import mujoco.viewer
import time
import json
import os
import numpy as np
from datetime import datetime

# ========== 模型定义 ==========
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
    <body name="scrubber" pos="0.18 0 0.12">
        <freejoint/>
        <geom type="mesh" mesh="brush_mesh" euler="0 90 0" rgba="1 0.8 0.8 1" mass="0.2"/>
        <site name="force_sensor" pos="0 0 0" size="0.01" type="sphere" rgba="0 1 0 1" group="3"/>
    </body>
</worldbody>
<sensor>
    <force name="scrubber_force" site="force_sensor"/>
</sensor>
</mujoco>
"""

model = mujoco.MjModel.from_xml_string(XML)
data = mujoco.MjData(model)

# 获取索引
force_sensor_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, "scrubber_force")
scrubber_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "scrubber")
scrubber_jnt_id = model.body_jntadr[scrubber_body_id]

# 自由度索引
qpos_x = model.jnt_qposadr[scrubber_jnt_id] + 0
qpos_y = model.jnt_qposadr[scrubber_jnt_id] + 1
qpos_z = model.jnt_qposadr[scrubber_jnt_id] + 2

# ========== 关键参数（可调整） ==========
COORD_FILE = 'C:/GCPD/face_coord.json'

# 校准参数（根据你的实际设备调整）
CALIB_OFFSET_X = 0.0  # X 轴偏移
CALIB_OFFSET_Y = 0.0  # Y 轴偏移
SIM_RANGE_X = 0.4    # X 轴仿真范围 (米)
SIM_RANGE_Y = 0.3    # Y 轴仿真范围 (米)
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480

# 滤波参数
ALPHA = 0.3          # 低通滤波系数 (0.1-0.5，越小越平滑但延迟越大)
DEAD_ZONE = 0.005    # 死区阈值 (小于此值不移动)

# 超时参数
TIMEOUT_MS = 500     # 坐标超时时间（毫秒）

# 状态变量
last_sim_x = 0.18    # 初始位置
last_sim_y = 0.0
last_update_time = None
face_lost_count = 0

print("=" * 50)
print("搓背机器人仿真系统 - 修复版")
print("=" * 50)
print(f"滤波系数：{ALPHA}")
print(f"死区阈值：{DEAD_ZONE}")
print(f"校准偏移：({CALIB_OFFSET_X}, {CALIB_OFFSET_Y})")
print("=" * 50)
print("仿真端已启动... 等待视觉坐标...")

def map_pixel_to_sim(px, py):
    """像素坐标 → 仿真世界坐标（带校准）"""
    # 归一化到 [0, 1]
    norm_x = px / CAMERA_WIDTH
    norm_y = py / CAMERA_HEIGHT
    
    # 映射到仿真范围（以中心为原点）
    sim_x = (norm_x - 0.5) * SIM_RANGE_X + CALIB_OFFSET_X
    sim_y = (norm_y - 0.5) * SIM_RANGE_Y + CALIB_OFFSET_Y
    
    return sim_x, sim_y

def lowpass_filter(current, target, alpha):
    """一阶低通滤波（指数平滑）"""
    return current + alpha * (target - current)

def apply_dead_zone(current, target, dead_zone):
    """应用死区阈值"""
    diff = target - current
    if abs(diff) < dead_zone:
        return current  # 变化太小，不移动
    return target

with mujoco.viewer.launch_passive(model, data) as viewer:
    # 初始化位置
    data.qpos[qpos_x] = last_sim_x
    data.qpos[qpos_y] = last_sim_y
    data.qpos[qpos_z] = 0.12  # 固定高度，保持接触
    
    while viewer.is_running():
        current_time = datetime.now()
        
        # 读取视觉坐标
        target_x, target_y = None, None
        
        if os.path.exists(COORD_FILE):
            try:
                with open(COORD_FILE, 'r') as f:
                    coord = json.load(f)
                
                # 检查时间戳（如果有）
                if 'timestamp' in coord:
                    coord_time = datetime.fromisoformat(coord['timestamp'])
                    age_ms = (current_time - coord_time).total_seconds() * 1000
                    if age_ms > TIMEOUT_MS:
                        print(f"⚠️  坐标超时 ({age_ms:.0f}ms)，使用上次有效值")
                        face_lost_count += 1
                    else:
                        px = coord.get('x')
                        py = coord.get('y')
                        if px is not None and py is not None:
                            target_x, target_y = map_pixel_to_sim(px, py)
                            face_lost_count = 0  # 重置丢失计数
                else:
                    # 没有时间戳，直接读取
                    px = coord.get('x')
                    py = coord.get('y')
                    if px is not None and py is not None:
                        target_x, target_y = map_pixel_to_sim(px, py)
                        face_lost_count = 0
                        
            except Exception as e:
                print(f"📁 读取出错：{e}")
                face_lost_count += 1
        
        # 处理坐标更新
        if target_x is not None and target_y is not None:
            # 1. 应用死区
            new_x = apply_dead_zone(last_sim_x, target_x, DEAD_ZONE)
            new_y = apply_dead_zone(last_sim_y, target_y, DEAD_ZONE)
            
            # 2. 应用低通滤波
            last_sim_x = lowpass_filter(last_sim_x, new_x, ALPHA)
            last_sim_y = lowpass_filter(last_sim_y, new_y, ALPHA)
            
            last_update_time = current_time
        else:
            # 人脸丢失，保持原位
            face_lost_count += 1
            if face_lost_count > 100:  # 约 1 秒后打印警告
                print("⚠️  人脸丢失，保持原位")
        
        # 更新机械臂位置
        data.qpos[qpos_x] = last_sim_x
        data.qpos[qpos_y] = last_sim_y
        # qpos_z 保持固定，确保接触
        
        # 物理步进
        mujoco.mj_step(model, data)
        
        # 力传感器读数
        force = data.sensordata[force_sensor_id*3 : force_sensor_id*3+3]
        if abs(force[2]) > 0.01:
            print(f"💪 接触力：{force[2]:.4f} N")
        
        viewer.sync()
        time.sleep(0.01)  # 约 100 FPS
