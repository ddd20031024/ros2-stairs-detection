import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge
import cv2
import math
from ultralytics import YOLO

class SensorFusionNode(Node):
    def __init__(self):
        super().__init__('yolo_lidar_fusion')
        self.bridge = CvBridge()
        
        # 自动加载官方轻量级模型（第一次运行会自动下载，很快）
        self.model = YOLO('/home/xpy/test/best_seg.pt') 
        self.get_logger().info('YOLO 模型加载完成！准备寻找水瓶...')
        
        self.latest_scan = None
        # 假设摄像头的水平视场角 (FOV) 为 60 度，这可以通过后续标定来修改
        self.fov = 60.0 

        # 订阅雷达数据
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        # 订阅摄像头数据
        self.img_sub = self.create_subscription(Image, '/image_raw', self.img_cb, 10)
        
    def scan_cb(self, msg):
        # 每次收到雷达数据，就把它存起来，供图像处理时查表用
        self.latest_scan = msg

    def img_cb(self, msg):
        if self.latest_scan is None:
            return # 如果还没收到雷达数据，先不处理图像
            
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        h, w, _ = cv_image.shape
        
        # 运行 YOLO 推理，classes=[39] 代表 COCO 数据集中的 bottle (水瓶)
        results = self.model.predict(source=cv_image, conf=0.5, verbose=False)
        result = results[0]
        
        annotated_frame = result.plot()
        
        if len(result.boxes) > 0:
            #把 GPU 里的深度学习数据，转化成 CPU 里普通的数字数组
            box = result.boxes[0].xyxy[0].cpu().numpy()
            x_center = int((box[0] + box[2]) / 2)
            y_center = int((box[1] + box[3]) / 2)
            
            # --- 核心逻辑：像素转角度 ---
            # 假设画面正中心是 0 度，左边是正角度，右边是负角度
            angle_deg = ((w / 2) - x_center) / w * self.fov
            
            # 将角度转换为雷达的 0-360 度范围
            if angle_deg < 0:
                angle_deg += 360.0
                
            # --- 核心逻辑：雷达查表 ---
            angle_rad = math.radians(angle_deg)
            # 计算这个角度在 ranges 数组中的索引
            index = int(angle_rad / self.latest_scan.angle_increment)
            
            if 0 <= index < len(self.latest_scan.ranges):
                dist = self.latest_scan.ranges[index]
                # 过滤掉无效值和盲区噪点
                if not math.isinf(dist) and not math.isnan(dist) and dist > 0.02:
                    # 在画面上用水瓶的中心点标出距离
                    text = f"Dist: {dist:.2f}m"
                    cv2.putText(annotated_frame, text, (x_center, y_center - 20), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                    self.get_logger().info(f'发现水瓶！像素 X={x_center}, 换算角度={angle_deg:.1f}度, 雷达测距={dist:.3f}米')
        
        cv2.imshow("Sensor Fusion Test", annotated_frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = SensorFusionNode()
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