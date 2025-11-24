import sklearn
import joblib
import os
import cv2
import numpy as np
import pytesseract  


class Recorte2number():
    def __init__(self):
        ruta = os.path.expanduser("~/Phoenyx/src/percepcion/percepcion/final_knn.pkl")
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
        """
        Detecta el color de la cartulina, extrae el número y devuelve una imagen:
            - Fondo negro
            - Número blanco, totalmente relleno
            - Tamaño 50x50 para KNN
        """

        try:
            # -----------------------------
            # DETECCIÓN DEL COLOR
            # -----------------------------
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

            lower_blue = np.array([100, 150, 50])
            upper_blue = np.array([140, 255, 255])

            lower_red1 = np.array([0, 150, 50])
            upper_red1 = np.array([10, 255, 255])
            lower_red2 = np.array([160, 150, 50])
            upper_red2 = np.array([180, 255, 255])

            mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
            mask_red = cv2.inRange(hsv, lower_red1, upper_red1) + cv2.inRange(hsv, lower_red2, upper_red2)

            blue_pixels = cv2.countNonZero(mask_blue)
            red_pixels = cv2.countNonZero(mask_red)

            if blue_pixels > red_pixels:
                color = "Azul"
                mask = mask_blue
            elif red_pixels > 0:
                color = "Rojo"
                mask = mask_red
            else:
                color = "Indefinido"
                mask = mask_blue + mask_red

            # -----------------------------
            # DETECTAR CARTULINA
            # -----------------------------
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if not contours:
                return None, color, None

            cartulina_contour = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(cartulina_contour)
            roi_cartulina = image[y:y+h, x:x+w]

            # -----------------------------
            # PROCESAR ROI DE LA CARTULINA
            # -----------------------------
            gray = cv2.cvtColor(roi_cartulina, cv2.COLOR_BGR2GRAY)

            # Quitamos bordes de la cartulina (evita falsos contornos)
            pad = int(min(w, h) * 0.05)
            pad = max(pad, 5)
            gray_crop = gray[pad:h-pad, pad:w-pad]

            # Threshold adaptativo → número blanco, fondo negro
            img_thresh = cv2.adaptiveThreshold(
                gray_crop, 255,
                cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                cv2.THRESH_BINARY_INV,
                21, 5
            )

            # Eliminar ruido
            kernel = np.ones((3,3), np.uint8)
            img_thresh = cv2.morphologyEx(img_thresh, cv2.MORPH_OPEN, kernel)

            # -----------------------------
            # CONTORNO DEL NÚMERO
            # -----------------------------
            contours_num, _ = cv2.findContours(img_thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if not contours_num:
                return None, color, None

            # Filtrar contornos irrelevantes
            max_area = (w * h) * 0.6
            min_area = 200

            contours_num = [c for c in contours_num if min_area < cv2.contourArea(c) < max_area]
            if not contours_num:
                return None, color, None

            numero_contour = max(contours_num, key=cv2.contourArea)
            x_n, y_n, w_n, h_n = cv2.boundingRect(numero_contour)

            numero_roi = img_thresh[y_n:y_n+h_n, x_n:x_n+w_n]

            # -----------------------------
            # RELLENO COMPLETO (FLOOD FILL)
            # -----------------------------
            flood = numero_roi.copy()
            hh, ww = flood.shape[:2]

            mask = np.zeros((hh+2, ww+2), np.uint8)
            cv2.floodFill(flood, mask, (0, 0), 255)

            flood_inv = cv2.bitwise_not(flood)
            img_filled = cv2.bitwise_or(numero_roi, flood_inv)

            # -----------------------------
            # IMAGEN FINAL 50x50
            # -----------------------------
            img_final = cv2.resize(img_filled, (50, 50))

            cv2.imshow("Imagen que entra al KNN", img_final)
            cv2.waitKey(1)

            # Bounding box en imagen original
            cv2.rectangle(
                image,
                (x + pad + x_n, y + pad + y_n),
                (x + pad + x_n + w_n, y + pad + y_n + h_n),
                (0, 255, 0), 2
            )

            # -----------------------------
            # PREDICCIÓN KNN
            # -----------------------------
            numero = self.obtener_knn_num(img_final)

            return numero, color, img_final

        except Exception as e:
            print(f"Error en obtener_colorYnum: {e}")
            return None, "Indefinido", None




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
