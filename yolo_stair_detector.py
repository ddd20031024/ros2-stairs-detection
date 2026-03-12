import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

class YoloDetector(Node):
    def __init__(self):
        super().__init__('yolo_stair_detector')
        
        # 初始化 cv_bridge
        self.bridge = CvBridge()
        
        # 加载你的 YOLOv8-seg 模型 (请替换为你实际的模型路径)
        # 如果你之前导出了 OpenVINO，这里可以填 'best_openvino_model/' 会更流畅
        self.model = YOLO('/home/xpy/test/best_seg.pt') 
        self.get_logger().info('YOLO 模型加载成功！')

        # 订阅摄像头的原始图像话题
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10)
        self.get_logger().info('正在监听 /image_raw 话题...')

    def image_callback(self, msg):
        try:
            # 将 ROS 2 的 Image 消息转换为 OpenCV 可以处理的图像格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'图像转换失败: {e}')
            return

        # 获取画面的宽度，用于后续计算偏移角
        height, width, _ = cv_image.shape

        # 运行 YOLO 推理
        # conf=0.5 表示只输出置信度大于 50% 的结果
        results = self.model.predict(source=cv_image, conf=0.5, verbose=False)

        # 提取第一个结果（当前帧）
        result = results[0]
        
        # 假设我们检测到了楼梯（有边框数据）
        if len(result.boxes) > 0:
            # 提取第一个检测框的坐标 [x_min, y_min, x_max, y_max]
            box = result.boxes[0].xyxy[0].cpu().numpy()
            
            # 计算楼梯在画面中的横向中心点 (X_center)
            x_center = int((box[0] + box[2]) / 2)
            y_center = int((box[1] + box[3]) / 2)
            
            self.get_logger().info(f'发现楼梯！中心点像素坐标: X={x_center}, Y={y_center}')
            
            # 在图像上画一个显眼的红点标记中心位置
            cv2.circle(cv_image, (x_center, y_center), 10, (0, 0, 255), -1)
        else:
            self.get_logger().info('画面中未检测到楼梯...')

        # 将 YOLO 画好掩码和边框的图像提取出来
        annotated_frame = result.plot()
        
        # 如果刚才画了红点，把红点也叠加到可视化画面上
        if len(result.boxes) > 0:
            cv2.circle(annotated_frame, (x_center, y_center), 10, (0, 0, 255), -1)

        # 在屏幕上实时显示处理后的画面
        cv2.imshow("YOLOv8 Stair Detection", annotated_frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = YoloDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()