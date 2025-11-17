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

from percepcion.Recorte2number import Recorte2number
from percepcion.Img2recorte import image2recorte


class BrainIntelNode(Node):
    def __init__(self):
        super().__init__('brain_intel_node')

        # ------------------ Parámetros (configurables) ------------------
        self.declare_parameters(namespace='', parameters=[
            ('numero_muestras', 10),
            ('show_gui', False),   # si quieres ver ventana OpenCV
            ('debug', False)       # si quieres que guarde imágenes
        ])
        try:
            self.numero_muestras = int(self.get_parameter('numero_muestras').value)
        except Exception:
            self.numero_muestras = 10

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
        self.numero_really = 5
        self.i = 0
        self._reset_deadline = None

        # Contador para frames perdidos (logado periódicamente)
        self._dropped_frames = 0
        self._last_drop_log = 0.0

        # Imagen para mostrar (solo una ventana OpenCV opcional)
        self.display_frame = None

        # Carpeta para guardar imágenes (solo si debug=True)
        self._img_dir = os.path.join('percepcion', 'imagenes')
        os.makedirs(self._img_dir, exist_ok=True)

        # ------------------ RealSense ------------------
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

        # ------------------ Timers ------------------
        # Timer de captura (20 Hz - cada 0.05s). Con wait_for_frames timeout reducido -> menos bloqueo.
        self.timer = self.create_timer(0.05, self.timer_callback)
        # Timer FSM (5 Hz)
        self.fsm_timer = self.create_timer(0.2, self.FSM)

    # -------------------- Lectura de cámara --------------------
    def timer_callback(self):
        if not self.camera_ready or self.pipe is None:
            return

        try:
            # Reducimos el timeout a 50 ms para evitar bloqueos largos.
            # Si la cámara no responde en ese tiempo, se continúa el loop rápidamente.
            try:
                frames = self.pipe.wait_for_frames(timeout_ms=50)
            except Exception as e:
                # contabilizamos frame perdido y seguimos; no queremos bloquear todo el nodo.
                self._dropped_frames += 1
                now = time.time()
                # log cada 2s si hay drops continuos
                if now - self._last_drop_log > 2.0:
                    self.get_logger().warning(f"Frames perdidos / retrasos cámara: {self._dropped_frames}")
                    self._last_drop_log = now
                return

            color_frame = frames.get_color_frame()
            if not color_frame:
                # si no hay color frame, contabilizar y salir
                self._dropped_frames += 1
                now = time.time()
                if now - self._last_drop_log > 2.0:
                    self.get_logger().warning(f"Frames sin color devueltos: {self._dropped_frames}")
                    self._last_drop_log = now
                return

            color_image = np.asanyarray(color_frame.get_data())

            # Publicar imagen ROS (intenta y registra fallo si lo hay)
            try:
                ros_img = self.br_rgb.cv2_to_imgmsg(color_image, encoding='bgr8')
                self.intel_pub.publish(ros_img)
            except Exception as e:
                self.get_logger().warning(f"Error publicando imagen ROS: {e}")

            # Procesamos si el estado lo permite
            if self.enable_muestras:
                self.tratar_recorte(color_image)

            # ------------------ Mostrar imagen (opcional) ------------------
            if self.show_gui:
                frame_to_show = self.display_frame if self.display_frame is not None else color_image
                try:
                    cv2.imshow("RGB", frame_to_show)
                    # waitKey(1) necesario para refrescar ventana; es mínimo pero opcional según show_gui
                    cv2.waitKey(1)
                except Exception as e:
                    # Sólo logueamos el problema y desactivamos la GUI para evitar microcongelaciones repetidas.
                    self.get_logger().warning(f"No se puede mostrar ventana OpenCV (desactivando GUI): {e}")
                    self.show_gui = False

        except Exception as e:
            self.get_logger().error(f"Error cámara: {e}")
            traceback.print_exc()

    # -------------------- Procesamiento de la imagen --------------------
    def tratar_recorte(self, image):
        try:
            res = self.converter.obtener_colorYnum(image)
        except Exception as e:
            self.get_logger().error(f"Error al obtener número y color: {e}")
            traceback.print_exc()
            return

        # Aceptamos que obtener_colorYnum pueda devolver None o tupla parcial
        if not res or not isinstance(res, tuple):
            return

        numero, color, img_thresh = res

        # número puede ser None si no se detectó nada
        if numero is not None:
            self.numeros.append(numero)

            print(f"[DETECCIÓN] Número detectado: {numero}, Color: {color}")


            # Guardar sólo si img_thresh es imagen válida y número distinto del real y debug=True
            if self.debug and numero != self.numero_really and isinstance(img_thresh, np.ndarray):
                filename = os.path.join(self._img_dir, f"{self.numero_really}_{numero}_{self.i}.png")
                try:
                    cv2.imwrite(filename, img_thresh)
                except Exception as e:
                    self.get_logger().warning(f"No se pudo escribir imagen {filename}: {e}")
            self.i += 1

            # Crear imagen con texto (no bloqueante)
            try:
                display_img = cv2.resize(image, (640, 480))
                display_text = f"Num: {numero}"
                if color:
                    display_text += f", Color: {color}"

                cv2.putText(display_img, display_text, (50, 50),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 0), 3)

                self.display_frame = display_img
            except Exception as e:
                self.get_logger().warning(f"Error formando display_frame: {e}")

        # Guardar color si viene definido
        if color:
            self.colores.append(color)

        self.conteo_muestras += 1

    # -------------------- Decisión final --------------------
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

    # -------------------- FSM --------------------
    def FSM(self):
        try:
            if self.estado == 1:      # RECOLECCIÓN
                self.enable_muestras = True
                if self.conteo_muestras >= self.numero_muestras or (time.time() - self.ini_time) >= 40:
                    self.estado = 2

            elif self.estado == 2:    # PROCESADO
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

                # acortamos la espera para que no pare demasiado el procesamiento
                self._reset_deadline = time.time() + 0.2
                self.estado = 3

            elif self.estado == 3:    # ESPERA NO BLOQUEANTE
                if time.time() >= self._reset_deadline:
                    self.numeros.clear()
                    self.colores.clear()
                    self.conteo_muestras = 0
                    self.ini_time = time.time()
                    self.display_frame = None
                    self.estado = 1
        except Exception as e:
            self.get_logger().error(f"Error en FSM: {e}")
            traceback.print_exc()

    # -------------------- Destructor --------------------
    def destroy_node(self):
        try:
            if getattr(self, 'pipe', None) is not None and self.camera_ready:
                try:
                    self.pipe.stop()
                except Exception as e:
                    self.get_logger().warning(f"Error parando RealSense: {e}")
        except Exception:
            pass

        try:
            cv2.destroyAllWindows()
        except Exception:
            pass

        try:
            super().destroy_node()
        except Exception:
            pass


# -------------------- MAIN --------------------
def main(args=None):
    rclpy.init(args=args)
    node = BrainIntelNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        rclpy.shutdown()


if __name__ == "__main__":
    main()
