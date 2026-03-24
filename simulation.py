#!/usr/bin/env python3
"""
搓背机器人仿真控制模块
基于 MuJoCo 物理引擎，读取视觉坐标控制机械臂运动
"""

import mujoco
import mujoco.viewer
import time
import json
import os
import numpy as np

# ========== 模型定义 ==========
XML = """
<mujoco>
<option gravity="0 0 0"/>
<asset>
    <mesh file="models/brush_mesh.obj" name="brush_mesh" scale="0.1 0.1 0.1"/>
    <mesh file="models/back_mesh.obj" name="back_mesh" scale="0.1 0.1 0.1"/>
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

def load_model():
    """加载模型"""
    try:
        model = mujoco.MjModel.from_xml_string(XML)
        data = mujoco.MjData(model)
        return model, data
    except Exception as e:
        print(f"模型加载失败，使用默认模型：{e}")
        # 如果模型文件不存在，使用简单几何体
        XML_SIMPLE = """
        <mujoco>
        <option gravity="0 0 0"/>
        <worldbody>
            <body name="back" pos="0.2 0 0.1">
                <geom type="box" size="0.15 0.2 0.05" rgba="0.8 0.7 0.6 1" mass="10"/>
            </body>
            <body name="scrubber" pos="0.18 0 0.12">
                <freejoint/>
                <geom type="sphere" size="0.03" rgba="1 0.8 0.8 1" mass="0.2"/>
                <site name="force_sensor" pos="0 0 0" size="0.01" type="sphere" rgba="0 1 0 1" group="3"/>
            </body>
        </worldbody>
        <sensor>
            <force name="scrubber_force" site="force_sensor"/>
        </sensor>
        </mujoco>
        """
        model = mujoco.MjModel.from_xml_string(XML_SIMPLE)
        data = mujoco.MjData(model)
        return model, data

def get_joint_ids(model):
    """获取关节 ID"""
    scrubber_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "scrubber")
    scrubber_jnt_id = model.body_jntadr[scrubber_body_id]
    
    qpos_x = model.jnt_qposadr[scrubber_jnt_id] + 0
    qpos_y = model.jnt_qposadr[scrubber_jnt_id] + 1
    qpos_z = model.jnt_qposadr[scrubber_jnt_id] + 2
    qpos_w = model.jnt_qposadr[scrubber_jnt_id] + 3
    qpos_xrot = model.jnt_qposadr[scrubber_jnt_id] + 4
    qpos_yrot = model.jnt_qposadr[scrubber_jnt_id] + 5
    qpos_zrot = model.jnt_qposadr[scrubber_jnt_id] + 6
    
    return {
        'x': qpos_x, 'y': qpos_y, 'z': qpos_z, 'w': qpos_w,
        'xrot': qpos_xrot, 'yrot': qpos_yrot, 'zrot': qpos_zrot
    }

def reset_pose(data, ids):
    """重置机械臂姿态"""
    # 强制直立
    data.qpos[ids['w']] = 1.0
    data.qpos[ids['xrot']] = 0.0
    data.qpos[ids['yrot']] = 0.0
    data.qpos[ids['zrot']] = 0.0
    # 默认起始位置
    data.qpos[ids['x']] = 0.15
    data.qpos[ids['y']] = 0.0
    data.qpos[ids['z']] = 0.12

def read_visual_coord(coord_file):
    """读取视觉坐标"""
    if not os.path.exists(coord_file):
        return None, None
    
    try:
        with open(coord_file, 'r') as f:
            coord = json.load(f)
        
        x_pix = coord.get('x')
        y_pix = coord.get('y')
        
        if x_pix is None or y_pix is None:
            return None, None
        
        # 映射到仿真世界（摄像头 640x480，范围 ±0.2 和 ±0.15）
        sim_x = (x_pix / 640) * 0.4 - 0.2
        sim_y = (y_pix / 480) * 0.3 - 0.15
        
        return sim_x, sim_y
    except Exception as e:
        print(f"读取出错：{e}")
        return None, None

def main():
    """主函数"""
    COORD_FILE = 'face_coord.json'
    
    # 加载模型
    model, data = load_model()
    joint_ids = get_joint_ids(model)
    
    # 重置姿态
    reset_pose(data, joint_ids)
    
    print("仿真端已启动... 等待视觉坐标...")
    print("按 Ctrl+C 退出")
    
    # 启动仿真
    with mujoco.viewer.launch_passive(model, data) as viewer:
        try:
            while viewer.is_running():
                # 强制保持直立姿态
                data.qpos[joint_ids['w']] = 1.0
                data.qpos[joint_ids['xrot']] = 0.0
                data.qpos[joint_ids['yrot']] = 0.0
                data.qpos[joint_ids['zrot']] = 0.0
                
                # 读取视觉坐标
                sim_x, sim_y = read_visual_coord(COORD_FILE)
                
                if sim_x is not None and sim_y is not None:
                    data.qpos[joint_ids['x']] = sim_x
                    data.qpos[joint_ids['y']] = sim_y
                
                # 物理步进
                mujoco.mj_step(model, data)
                
                # 检测接触力
                force_sensor_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, "scrubber_force")
                force = data.sensordata[force_sensor_id*3 : force_sensor_id*3+3]
                
                if abs(force[2]) > 0.01:
                    print(f"接触力：{force[2]:.4f} N")
                
                viewer.sync()
                time.sleep(0.01)
                
        except KeyboardInterrupt:
            print("\n仿真已停止")

if __name__ == '__main__':
    main()
