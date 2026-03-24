#!/usr/bin/env python3
"""
搓背机器人视觉模块
基于 OpenCV 的人脸检测，输出坐标供仿真模块使用
"""

import cv2
import json
import os

COORD_FILE = 'face_coord.json'

def main():
    """主函数"""
    # 启动时删除旧坐标文件
    if os.path.exists(COORD_FILE):
        os.remove(COORD_FILE)
    
    # 加载人脸检测模型
    face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
    
    # 打开摄像头
    cap = cv2.VideoCapture(0)
    
    if not cap.isOpened():
        print("摄像头无法打开")
        return
    
    print("视觉端已启动... 按 'q' 退出")
    
    try:
        while cap.isOpened():
            ret, frame = cap.read()
            
            if not ret:
                break
            
            # 转为灰度图
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            
            # 人脸检测
            faces = face_cascade.detectMultiScale(gray, 1.1, 4)
            
            face_x, face_y = None, None
            
            # 绘制检测结果
            for (x, y, w, h) in faces:
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                face_x = x + w // 2
                face_y = y + h // 2
                cv2.circle(frame, (face_x, face_y), 3, (0, 0, 255), -1)
            
            # 转换为 Python int（避免 numpy 类型问题）
            if face_x is not None:
                face_x = int(face_x)
            if face_y is not None:
                face_y = int(face_y)
            
            # 写入坐标文件
            data = {'x': face_x, 'y': face_y}
            
            with open(COORD_FILE, 'w') as f:
                json.dump(data, f)
            
            # 显示画面
            cv2.imshow('Face Detection', frame)
            
            # 按 'q' 退出
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    
    except KeyboardInterrupt:
        print("\n视觉已停止")
    
    finally:
        # 清理资源
        cap.release()
        cv2.destroyAllWindows()
        
        # 删除坐标文件
        if os.path.exists(COORD_FILE):
            os.remove(COORD_FILE)

if __name__ == '__main__':
    main()
