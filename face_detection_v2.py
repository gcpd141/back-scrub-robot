import cv2
import json
import os
from datetime import datetime

COORD_FILE = 'C:/GCPD/face_coord.json'
TEMP_FILE = COORD_FILE + '.tmp'

# 启动时删除旧文件
if os.path.exists(COORD_FILE):
    try:
        os.remove(COORD_FILE)
    except:
        pass

face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

print("=" * 50)
print("搓背机器人视觉系统 - 修复版")
print("=" * 50)

# 尝试打开摄像头
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("❌ 摄像头无法打开，请检查：")
    print("   1. 摄像头是否已连接")
    print("   2. 是否有其他程序占用摄像头")
    print("   3. 摄像头权限是否已授予")
    exit(1)

# 设置分辨率
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cap.set(cv2.CAP_PROP_FPS, 30)

print("✅ 摄像头已打开")
print(f"✅ 分辨率：{cap.get(cv2.CAP_PROP_FRAME_WIDTH)}x{cap.get(cv2.CAP_PROP_FRAME_HEIGHT)}")
print("✅ 按 'q' 退出")
print("=" * 50)

frame_count = 0
detect_count = 0
last_print_time = datetime.now()

while cap.isOpened():
    ret, frame = cap.read()
    
    if not ret:
        print("⚠️ 无法读取摄像头画面")
        frame_count += 1
        continue
    
    frame_count += 1
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # 人脸检测
    faces = face_cascade.detectMultiScale(gray, 1.1, 4)
    
    face_x, face_y = None, None
    
    for (x, y, w, h) in faces:
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        face_x = x + w // 2
        face_y = y + h // 2
        cv2.circle(frame, (face_x, face_y), 5, (0, 0, 255), -1)
        detect_count += 1
    
    # 写入坐标文件（带时间戳）
    data = {
        'x': int(face_x) if face_x is not None else None,
        'y': int(face_y) if face_y is not None else None,
        'timestamp': datetime.now().isoformat(),
        'frame_count': frame_count
    }
    
    # 原子写入（避免读写冲突）
    try:
        with open(TEMP_FILE, 'w') as f:
            json.dump(data, f)
        os.replace(TEMP_FILE, COORD_FILE)
    except Exception as e:
        print(f"⚠️ 写入失败：{e}")
    
    # 显示检测统计（每 2 秒打印一次）
    current_time = datetime.now()
    if (current_time - last_print_time).total_seconds() > 2:
        detection_rate = (detect_count / frame_count * 100) if frame_count > 0 else 0
        status = "✅ 检测到人脸" if face_x is not None else "⏳ 未检测到人脸"
        print(f"{status} | 帧率：{frame_count/ (current_time - last_print_time).total_seconds():.1f} FPS | 检测率：{detection_rate:.1f}%")
        last_print_time = current_time
        detect_count = 0
        frame_count = 0
    
    # 显示状态
    cv2.putText(frame, f'Face: {"YES" if face_x else "NO"}', (10, 30), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0) if face_x else (0, 0, 255), 2)
    
    cv2.imshow('Face Detection - Press Q to Exit', frame)
    
    # 检测 'q' 键退出
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

# 清理坐标文件
if os.path.exists(COORD_FILE):
    try:
        os.remove(COORD_FILE)
    except:
        pass

print("\n✅ 视觉系统已退出")
