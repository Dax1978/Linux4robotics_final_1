from typing import List
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2
import time
import os
import glob

# Определяю класс CaptureCameraNode, являющийся ROS2-нодой для захвата изображений с камеры
class CaptureCameraNode(Node):
    def __init__(self) -> None:
        # Инициирую родительский класс Node с именем 'capture_camera'
        super().__init__('capture_camera')

        # Создаю объект CvBridge для преобразования изображений между OpenCV и ROS
        self.bridge = CvBridge()

        # Создаю издателей для публикации изображений и текстовых сообщений
        self.publisher = self.create_publisher(Image, 'camera/image', 10)
        self.text_publisher = self.create_publisher(String, 'camera/info', 10)

        # Определяю путь к устройству камеры
        # path = "/"
        self.cap = cv2.VideoCapture(0)
        print("Camera created", self.cap)

        # Таймер, вызывающий функцию capture_and_publish каждую секунду
        self.timer = self.create_timer(1.0, self.capture_and_publish)

        # Путь для сохранения изображений
        self.image_save_path = "/home/dax/images"
        # Если такой папки нет, создаю ее
        os.makedirs(self.image_save_path, exist_ok=True)

        # Таймер, вызывающий функцию save_image каждые 5 секунд
        self.save_timer = self.create_timer(5.0, self.save_image)

    # Метод захвата и публикации изображения
    def capture_and_publish(self):
        # Захватываю изображение с камеры
        ret, frame = self.cap.read()
        if ret:
            # Вывожу текущее время захвата в консоль
            print("Last time of get image:", time.ctime())

            # Преобразую изображение в формат ROS и публикую
            msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            self.publisher.publish(msg)

            # Готовлю и пбликую текстовое сообщение с временем захвата изображения
            msg = String()
            msg.data = f"Last time of get image: {time.ctime()}"
            self.text_publisher.publish(msg)

    # Метод сохранения изоражения и обеспечение хранения только 20 последних изображений
    def save_image(self):
        ret, frame = self.cap.read()
        if ret:
            # Сохраняем изображение с учетом временной метки
            timestamp = time.strftime("%Y%m%d-%H%M%S")
            filename = os.path.join(self.image_save_path, f"image_{timestamp}.jpg")
            cv2.imwrite(filename, frame)
            print(f"Saved image to {filename}")

            # Получаем список всех сохраненных изображений
            all_images = glob.glob(os.path.join(self.image_save_path, "*.jpg"))
            # Сортирую по дате создания
            sorted_images = sorted(all_images, key=os.path.getmtime)
            # Удаляю старые
            while len(sorted_images) > 20:
                os.remove(sorted_images[0])
                del sorted_images[0]

        
def main(args=None):
    # Инициализация ROS
    rclpy.init(args=args)
    # Создаю объект ноды
    capture_camera = CaptureCameraNode()
    # Запускаю цикл обработки ROS
    rclpy.spin(capture_camera)


if __name__ == '__main__':
    main()

