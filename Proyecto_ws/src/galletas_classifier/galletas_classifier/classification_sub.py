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
        self.delay_seconds = 1  # Delay de 1 segundo entre cada clasificación

        # Inicialización del escritor de video
        self.video_writer = None
        self.frame_width = 640  # Ajusta esto a la resolución de tu video
        self.frame_height = 480  # Ajusta esto a la resolución de tu video
        self.video_filename = 'classified_output.avi'
        self.fourcc = cv2.VideoWriter_fourcc(*'XVID')
        self.fps = 10

    def listener_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')  # Convertir la imagen ROS a una imagen OpenCV
        results = self.model.predict(frame, imgsz=800, conf=0.3, save=False, show=True)  # Realizar la detección usando YOLOv8

        # Contadores de galletas por frame
        frame_quemadas = 0
        frame_partidas = 0
        frame_buenas = 0

        if results[0].boxes is not None:
            for box in results[0].boxes:
                class_id = int(box.cls)  # Obtener el ID de la clase de la detección
                self.counts[class_id] += 1  # Actualizar el contador de la clase correspondiente

                # Dibujar las cajas y etiquetas en el frame
                box_coords = box.xyxy[0].tolist()  # Convertir las coordenadas de la caja a lista
                cv2.rectangle(frame, (int(box_coords[0]), int(box_coords[1])), (int(box_coords[2]), int(box_coords[3])), (255, 0, 0), 2)
                label = f'Class {class_id}'
                cv2.putText(frame, label, (int(box_coords[0]), int(box_coords[1]) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (36, 255, 12), 2)

                if class_id == 0:
                    frame_partidas += 1
                    self.publisher_.publish(String(data='Defectuosa: Quebrada'))
                elif class_id == 1:
                    frame_quemadas += 1
                    self.publisher_.publish(String(data='Defectuosa: Quemada'))
                elif class_id == 2:
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

        # Inicializar el escritor de video si no está inicializado
        if self.video_writer is None:
            self.frame_width = frame.shape[1]
            self.frame_height = frame.shape[0]
            self.video_writer = cv2.VideoWriter(self.video_filename, self.fourcc, self.fps, (self.frame_width, self.frame_height))

        # Escribir el frame con las etiquetas en el archivo de video
        self.video_writer.write(frame)

    def destroy_node(self):
        if self.video_writer is not None:
            self.video_writer.release()  # Liberar el recurso del escritor de video
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ClassificationSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()