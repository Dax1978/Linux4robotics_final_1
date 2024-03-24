import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class ImageReaderNode(Node):

    def __init__(self):
        # Инициализирую родительский класс Node с именем 'image_reader_node'
        super().__init__('image_reader_node')

        # Создаю объект CvBridge для преобразования изображений между OpenCV и ROS
        self.bridge = CvBridge()

        # Подписываюсь на тему 'camera/image' для чтения изображений
        self.subscription = self.create_subscription(
            Image,
            'camera/image',
            self.image_callback,
            10
        )
        self.subscription

    def image_callback(self, msg):
        # Преобразую изображение из формата ROS в формат OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # Получаю разрешение изображения
        height, width, _ = cv_image.shape
        print(f"Image resolution: {width}x{height}")

    
def main(args=None):
    rclpy.init(args=args)
    image_reader = ImageReaderNode()
    rclpy.spin(image_reader)


if __name__ == '__main__':
    main()