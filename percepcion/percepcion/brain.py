import os
import time
import traceback
from collections import Counter

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from cv_bridge import CvBridge
import pyrealsense2 as rs
import threading   # ← Para el hilo continuo

from percepcion.Recorte2number import Recorte2number
from percepcion.Img2recorte import image2recorte


class BrainIntelNode(Node):
    def __init__(self):
        super().__init__('brain_intel_node')

        # ------------------ Parámetros ------------------
        self.declare_parameters(namespace='', parameters=[
            ('numero_muestras', 10),
            ('show_gui', False),
            ('debug', False)
        ])

        self.numero_muestras = int(self.get_parameter('numero_muestras').value)
        self.show_gui = bool(self.get_parameter('show_gui').value)
        self.debug = bool(self.get_parameter('debug').value)

        self.get_logger().info(f"Número de muestras: {self.numero_muestras} (show_gui={self.show_gui}, debug={self.debug})")

        # ------------------ Publicadores ------------------
        self.pub_vueltas = self.create_publisher(Int32, '/num_vueltas', 10)
        self.intel_pub = self.create_publisher(Image, 'rgb_frame', 10)
        self.br_rgb = CvBridge()

        # ------------------ Procesadores ------------------
        self.converter = Recorte2number()
        self.image_processor = image2recorte()

        # ------------------ Variables internas ------------------
        self.numeros = []
        self.colores = []
        self.conteo_muestras = 0
        self.estado = 1
        self.enable_muestras = True
        self.numero_final = 0
        self.color_final = ""
        self.ini_time = time.time()
        self.last_time = time.time()   
        self.fps = 0.0                 
        self.numero_really = 5
        self.i = 0
        self._reset_deadline = None

        # Carpeta debug
        self._img_dir = os.path.join('percepcion', 'imagenes')
        os.makedirs(self._img_dir, exist_ok=True)

        # ------------------ Inicializar RealSense ------------------
        try:
            self.pipe = rs.pipeline()
            self.cfg = rs.config()
            self.cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
            self.pipe.start(self.cfg)
            self.camera_ready = True
            self.get_logger().info("RealSense inicializada.")
        except Exception as e:
            self.get_logger().error(f"RealSense NO conectada: {e}")
            traceback.print_exc()
            self.camera_ready = False
            self.pipe = None

        # ------------------ Hilo de captura continua ------------------
        # Flag para parar el hilo al destruir el nodo
        self._running = True

        # El hilo llama a self.camera_loop()
        self.capture_thread = threading.Thread(
            target=self.camera_loop,
            daemon=True
        )
        self.capture_thread.start()
        self.get_logger().info("Hilo de captura iniciado.")

        # ------------------ Timer para la FSM ------------------
        # Solo necesitamos esto, no timers de cámara
        self.fsm_timer = self.create_timer(0.001, self.FSM)

    # ============================================================================================
    #                                   LOOP CONTINUO REALSENSE
    # ============================================================================================
    def camera_loop(self):
        
        if not self.camera_ready:
            self.get_logger().error("No se puede iniciar camera_loop: cámara no lista")
            return

        while self._running:
            try:
                # Espera frame con timeout pequeño (evita congelaciones)
                frames = self.pipe.wait_for_frames(timeout_ms=1000)
                color_frame = frames.get_color_frame()
                if not color_frame:
                    continue

                # Convertir a array np
                color_image = np.asanyarray(color_frame.get_data())

                # ---------------------------------------------------------------------
                # CALCULAR FPS
                # ---------------------------------------------------------------------
                now = time.time()
                dt = now - self.last_time
                if dt > 0:
                    self.fps = 1.0 / dt
                self.last_time = now

                # ---------------------------------------------------------------------
                # TEXTO SOBRE LA IMAGEN
                # Muestra: Color: X // Numero: Y // FPS: Z
                # ---------------------------------------------------------------------
                mensaje = f"Color: {self.color_final} // Numero: {self.numero_final} // FPS: {self.fps:.1f}"

                cv2.putText(
                    color_image,
                    mensaje,
                    (20, 40),                       # posición
                    cv2.FONT_HERSHEY_SIMPLEX,       # fuente
                    0.8,                            # tamaño
                    (0, 255, 0),                    # verde
                    2                               # grosor
                )
                # ---------------------------------------------------------------------

                # Publicar imagen ROS
                try:
                    ros_img = self.br_rgb.cv2_to_imgmsg(color_image, encoding='bgr8')
                    self.intel_pub.publish(ros_img)
                except Exception as e:
                    self.get_logger().warning(f"Error publicando imagen ROS: {e}")

                # Procesar muestras SOLO si la FSM lo permite
                if self.enable_muestras:
                    self.tratar_recorte(color_image)

                # Mostrar ventana si el usuario lo activó
                if self.show_gui:
                    cv2.imshow("RGB", color_image)
                    cv2.waitKey(1)

            except Exception as e:
                self.get_logger().error(f"Error en camera_loop: {e}")
                traceback.print_exc()

        self.get_logger().info("camera_loop detenido correctamente.")

    # ============================================================================================
    #                                   PROCESAMIENTO DE IMAGEN
    # ============================================================================================
    def tratar_recorte(self, image):
        """
        Procesa un frame para extraer número y color.
        """
        try:
            res = self.converter.obtener_colorYnum(image)
        except Exception as e:
            self.get_logger().error(f"Error al obtener número y color: {e}")
            traceback.print_exc()
            return

        if not res or not isinstance(res, tuple):
            return

        numero, color, img_thresh = res

        if numero is not None:
            self.numeros.append(numero)
            print(f"[DETECCIÓN] Número detectado: {numero}, Color: {color}")

            if self.debug and numero != self.numero_really and isinstance(img_thresh, np.ndarray):
                filename = os.path.join(self._img_dir, f"{self.numero_really}_{numero}_{self.i}.png")
                try:
                    cv2.imwrite(filename, img_thresh)
                except Exception as e:
                    self.get_logger().warning(f"No se pudo escribir imagen {filename}: {e}")
            self.i += 1

        if color:
            self.colores.append(color)

        self.conteo_muestras += 1

    # ============================================================================================
    #                                   DECISIÓN FINAL
    # ============================================================================================
    def decision_making(self):
        if not self.numeros:
            return 0, "Distractorio"

        cnt = Counter(self.numeros)
        numero = cnt.most_common(1)[0][0]

        colores = self.colores or []
        counts = {
            "Rojo": colores.count("Rojo"),
            "Azul": colores.count("Azul"),
            "Distractorio": len(colores) - colores.count("Rojo") - colores.count("Azul")
        }
        color = max(counts, key=counts.get)
        return numero, color

    # ============================================================================================
    #                                   FSM DEL SISTEMA
    # ============================================================================================
    def FSM(self):
        """
        Lógica de la máquina de estados.
        Solo se ejecuta desde un timer (no bloqueante).
        """
        try:
            if self.estado == 1:  # RECOLECCIÓN
                self.enable_muestras = True
                if self.conteo_muestras >= self.numero_muestras or (time.time() - self.ini_time) >= 40:
                    self.estado = 2

            elif self.estado == 2:  # PROCESADO
                self.enable_muestras = False
                self.numero_final, self.color_final = self.decision_making()

                msg = Int32()
                if self.color_final == "Azul":
                    msg.data = int(self.numero_final)
                elif self.color_final == "Rojo":
                    msg.data = -int(self.numero_final)
                else:
                    msg.data = 0

                self.pub_vueltas.publish(msg)
                self._reset_deadline = time.time() + 0.2
                self.estado = 3

            elif self.estado == 3:  # ESPERA
                if time.time() >= self._reset_deadline:
                    self.numeros.clear()
                    self.colores.clear()
                    self.conteo_muestras = 0
                    self.ini_time = time.time()
                    self.estado = 1

        except Exception as e:
            self.get_logger().error(f"Error en FSM: {e}")
            traceback.print_exc()

    # ============================================================================================
    #                                   DESTRUCTOR
    # ============================================================================================
    def destroy_node(self):
        """
        Se ejecuta al cerrar ROS2.
        Para el hilo, cierra cámara y cierra ventanas.
        """
        self.get_logger().info("Finalizando nodo...")

        # Pedimos que el hilo termine
        self._running = False
        time.sleep(0.1)

        # Parar RealSense
        try:
            if self.camera_ready and self.pipe:
                self.pipe.stop()
        except Exception as e:
            self.get_logger().warning(f"Error parando RealSense: {e}")

        # Cerrar ventanas
        try:
            cv2.destroyAllWindows()
        except:
            pass

        try:
            super().destroy_node()
        except:
            pass


# ------------------------------------------ MAIN ------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = BrainIntelNode()
    try:
        rclpy.spin(node)   # ROS2 ejecuta timers y callbacks
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
