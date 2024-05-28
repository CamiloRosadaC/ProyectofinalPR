import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import time


class ClassificationSubscriber(Node):
    def __init__(self):
        super().__init__('classification_subscriber')
        self.subscription = self.create_subscription(Image, 'webcam_image', self.listener_callback, 10)
        self.publisher_ = self.create_publisher(String, 'defective_biscuits', 10)
        self.bridge = CvBridge()
        self.model = YOLO('/home/camilo/Proyecto_ws/src/galletas_classifier/galletas_classifier/best.pt')  # Asegúrate de tener el modelo entrenado en la ruta correcta
        self.counts = {0: 0, 1: 0, 2: 0}  # Inicialización de contadores por ID de clase
        self.delay_seconds = 2  # Delay de 2 segundo entre cada clasificación

    def listener_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')  # Convertir la imagen ROS a una imagen OpenCV
        results = self.model.predict(frame, imgsz=800, conf=0.3, save=True, show=True)  # Realizar la detección usando YOLOv8

        # Contadores de galletas por frame
        frame_quemadas = 0
        frame_partidas = 0
        frame_buenas = 0

        if results[0].boxes is not None:
            for box in results[0].boxes:
                class_id = int(box.cls)  # Obtener el ID de la clase de la detección
                self.counts[class_id] += 1  # Actualizar el contador de la clase correspondiente

                if class_id == 0:
                    frame_partidas += 1
                    self.publisher_.publish(String(data='Defectuosa: Quebrada'))
                elif class_id == 1:
                    frame_quemadas += 1
                    self.publisher_.publish(String(data='Defectuosa: Quemada'))
                elif class_id == 0:
                    frame_buenas += 1
                    

                # Agregar delay entre cada clasificación
                time.sleep(self.delay_seconds)

        # Publicar los resultados agregados del frame
        self.publisher_.publish(String(data=f'Frame - Quemadas = {frame_quemadas}'))
        self.publisher_.publish(String(data=f'Frame - Quebradas = {frame_partidas}'))
        self.publisher_.publish(String(data=f'Frame - Buenas = {frame_buenas}'))

        # Publicar el conteo total acumulado
        self.publisher_.publish(String(data=f'Total - Quemadas = {self.counts[2]}'))
        self.publisher_.publish(String(data=f'Total - Quebradas = {self.counts[1]}'))
        self.publisher_.publish(String(data=f'Total - Buenas = {self.counts[0]}'))

        # Mostrar el conteo actualizado en la consola para depuración
        self.get_logger().info(f'Conteo acumulativo de galletas: {self.counts}')

def main(args=None):
    rclpy.init(args=args)
    node = ClassificationSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()