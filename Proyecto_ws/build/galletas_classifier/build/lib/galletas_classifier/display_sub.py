#import rclpy
#from rclpy.node import Node
#from std_msgs.msg import String

#class DisplaySubscriber(Node):
#    def __init__(self):
#        super().__init__('display_subscriber')
#        self.subscription = self.create_subscription(String, 'defective_biscuits', self.listener_callback, 10)

#    def listener_callback(self, msg):
#        self.get_logger().info(msg.data)
#        print("Mensaje en pantalla:", msg.data)

#def main(args=None):
#    rclpy.init(args=args)
#    node = DisplaySubscriber()
#    rclpy.spin(node)
#    node.destroy_node()
#    rclpy.shutdown()

#if __name__ == '__main__':
#    main()

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

class DisplaySubscriber(Node):
    def __init__(self):
        super().__init__('display_subscriber')
        self.subscription = self.create_subscription(String, 'defective_biscuits', self.listener_callback, 10)
        
        # Configuración del puerto serie para la conexión con Arduino
        self.arduino = serial.Serial('/dev/ttyACM0', 9600)  # Asegúrate de que el puerto es correcto
        self.arduino.timeout = 1  # Timeout de lectura

    def listener_callback(self, msg):
        self.get_logger().info(msg.data)
        print("Mensaje en pantalla:", msg.data)

        # Enviar mensaje al Arduino basado en la clasificación de las galletas
        if "Defectuosa" in msg.data:
            self.arduino.write(b'1')  # Enviar '1' si la galleta es defectuosa
        else:
            self.arduino.write(b'0')  # Enviar '0' si la galleta es buena

def main(args=None):
    rclpy.init(args=args)
    node = DisplaySubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()