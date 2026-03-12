import cv2

# 0 通常代表系统的第一个摄像头（也就是 /dev/video0）
# 如果运行报错，可以尝试把 0 改成 2 或其他 ls 出来的数字
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("无法打开摄像头，请检查端口号或权限！")
    exit()

print("摄像头已成功打开！按 'q' 键退出。")

while True:
    # 逐帧读取图像
    ret, frame = cap.read()
    
    if not ret:
        print("无法接收画面 (可能视频流已断开)。")
        break

    # 显示画面
    cv2.imshow('Camera Test', frame)

    # 按下 'q' 键退出循环
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 释放资源并关闭窗口
cap.release()
cv2.destroyAllWindows()
cv2.destroyAllWindows()