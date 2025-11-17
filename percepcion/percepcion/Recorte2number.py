import sklearn
import joblib
import os
import cv2
import numpy as np
import pytesseract  


class Recorte2number():
    def __init__(self):
        ruta = os.path.expanduser("~/Phoenyx/src/percepcion/percepcion/modelo_knn(3).pkl")
        self.knn = joblib.load(ruta)
        self.prev_num = 0


    def obtener_num(self, image, log_level=0):
        """Preprocesa la imagen y extrae un número usando OCR."""
        try:
            image = cv2.bitwise_not(image)
            config = '--psm 10 -c tessedit_char_whitelist=12346789'
            number = pytesseract.image_to_string(image, config=config).strip()

            data_list = pytesseract.image_to_data(image, config=config, output_type=pytesseract.Output.DICT)
            confidences = [c for c in data_list['conf'] if c != -1]

            average_confidence = sum(confidences) / len(confidences) if len(confidences) > 0 else 0

            if not number or average_confidence < 1:
                return None

            return int(number[0])

        except Exception as e:
            print(f"Ocurrió un error: {e}")
            return None


    def detectar_color_bgr(self, numero_cuadrado):
        """Detecta la probabilidad de ser rojo o azul basándose en los canales BGR."""
        bgr_image = numero_cuadrado

        avg_b = np.mean(bgr_image[:, :, 0])  # azul
        avg_g = np.mean(bgr_image[:, :, 1])  # verde (antes ponía rojo → ERROR)
        avg_r = np.mean(bgr_image[:, :, 2])  # rojo

        max_value = max(avg_b, avg_g, avg_r)

        if max_value == avg_b and avg_g < 130 and avg_r < 130:
            return "Azul"
        elif max_value == avg_r and avg_g < 130 and avg_b < 130:
            return "Rojo"
        else:
            return "Indefinido"


    def obtener_knn_num(self, img_thresh):
        # Si pide la Knn 784 es un 28 x 28, si pide 2500 es 50 x 50
        img_resized = cv2.resize(img_thresh, (50, 50), interpolation=cv2.INTER_AREA)
        img_flat = img_resized.reshape(1, -1)
        return self.knn.predict(img_flat)[0]


    def obtener_colorYnum(self, image):
        color = self.detectar_color_bgr(image)

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        umbral = 150
        if color == "Rojo":
            umbral = 130
        elif color == "Azul":
            umbral = 140
        else:
            umbral = 120

        _, img_thresh = cv2.threshold(gray, umbral, 255, cv2.THRESH_BINARY)

        frame_thickness = 10
        img_thresh[-frame_thickness:, :] = 0
        img_thresh[:, :frame_thickness] = 0
        img_thresh[:frame_thickness, :] = 0
        img_thresh[:, -frame_thickness:] = 0

        img_bb = self.bounding_box(img_thresh)
        if img_bb is None:
            return None, color, img_thresh

        contornos, jerarquia = cv2.findContours(img_bb, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)

        img_smooth = np.zeros_like(img_bb)
        new_image = img_smooth.copy()
        contornos_vacios = []

        for i, contorno in enumerate(contornos):
            epsilon = 0.01 * cv2.arcLength(contorno, True)
            contorno_suavizado = cv2.approxPolyDP(contorno, epsilon, True)

            if jerarquia[0][i][3] == -1:
                cv2.drawContours(new_image, [contorno_suavizado], -1, (255), thickness=cv2.FILLED)
            else:
                contornos_vacios.append(contorno_suavizado)

        for contorno in contornos_vacios:
            cv2.drawContours(new_image, [contorno], -1, (0), thickness=cv2.FILLED)

        img_final = cv2.resize(new_image, (50, 50))

        numero = self.obtener_knn_num(img_final)
        return numero, color, img_final


    def ordenar_puntos_bounding_box(self, puntos):
        suma = puntos.sum(axis=1)
        dif = np.diff(puntos, axis=1)

        ordenados = np.zeros((4, 2), dtype="float32")
        ordenados[0] = puntos[np.argmin(suma)]
        ordenados[2] = puntos[np.argmax(suma)]
        ordenados[1] = puntos[np.argmin(dif)]
        ordenados[3] = puntos[np.argmax(dif)]
        return ordenados


    def bounding_box(self, binaria):
        contornos, _ = cv2.findContours(binaria, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contornos:
            return None

        cont = max(contornos, key=cv2.contourArea)
        rect = cv2.minAreaRect(cont)
        centro, size, angulo = rect
        ancho_rect, alto_rect = size

        if ancho_rect > alto_rect:
            alto_rect, ancho_rect = ancho_rect, alto_rect
            angulo += 90

        lado_cuadrado = alto_rect

        rect_cuadrado = (centro, (lado_cuadrado, lado_cuadrado), angulo)
        box_cuadrado = cv2.boxPoints(rect_cuadrado)
        box_cuadrado = np.intp(box_cuadrado)

        destino = np.array([
            [0, 0],
            [lado_cuadrado - 1, 0],
            [lado_cuadrado - 1, lado_cuadrado - 1],
            [0, lado_cuadrado - 1]
        ], dtype="float32")

        origen = self.ordenar_puntos_bounding_box(np.float32(box_cuadrado))
        M = cv2.getPerspectiveTransform(origen, destino)
        enderezada = cv2.warpPerspective(binaria, M, (int(lado_cuadrado), int(lado_cuadrado)))

        enderezada = cv2.resize(enderezada, (50, 50))
        return self.suavizar_numero(enderezada)


    def suavizar_numero(self, img_thresh):
        contornos, jerarquia = cv2.findContours(img_thresh, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
        img_smooth = np.zeros_like(img_thresh)

        new_image = img_smooth.copy()
        contornos_vacios = []

        for i, contorno in enumerate(contornos):
            epsilon = 0.01 * cv2.arcLength(contorno, True)
            cont_suave = cv2.approxPolyDP(contorno, epsilon, True)

            if jerarquia[0][i][3] == -1:
                cv2.drawContours(new_image, [cont_suave], -1, (255), thickness=cv2.FILLED)
            else:
                contornos_vacios.append(cont_suave)

        for contorno in contornos_vacios:
            cv2.drawContours(new_image, [contorno], -1, (0), thickness=cv2.FILLED)

        return new_image
