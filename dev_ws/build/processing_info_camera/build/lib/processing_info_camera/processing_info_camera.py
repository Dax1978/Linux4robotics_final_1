import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class InfoReaderNode(Node):

    def __init__(self):
        # Инициализирую родительский класс Node с именем 'info_reader_node'
        super().__init__('info_reader_node')

        # Подписываюсь на тему 'camera/image' для чтения текстовых сообщений
        self.subscription = self.create_subscription(
            String,
            'camera/info',
            self.info_callback,
            10
        )
        self.subscription

    def info_callback(self, msg):
        # Вывожу полученное текстовое сообщение в консоль
        print(f"Received info: {msg.data}")


def main(args=None):
    rclpy.init(args=args)
    info_reader = InfoReaderNode()
    rclpy.spin(info_reader)


if __name__ == '__main__':
    main()