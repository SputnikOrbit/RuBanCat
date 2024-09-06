
import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class VideoPublisherNode(Node):
    def __init__(self):
        super().__init__('video_publisher_node')

        # 使用 CvBridge 将 OpenCV 图像转换为 ROS2 图像消息
        self.bridge = CvBridge()

        # 创建发布者，将图像发布到 '/video_frames' 话题
        self.publisher_ = self.create_publisher(Image, '/image', 10)

        # 打开视频文件或摄像头，0 表示使用默认摄像头
        self.cap = cv2.VideoCapture(4)

        # 设置定时器以固定频率读取和发布视频帧
        timer_period = 0.05  # 以秒为单位，10 Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()

        if ret:
            # 将 OpenCV 图像转换为 ROS2 图像消息
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')

            # 发布图像
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing video frame')
        else:
            self.get_logger().warn('Failed to capture frame')

    def destroy_node(self):
        # 在销毁节点时释放摄像头
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = VideoPublisherNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
