#!/usr/bin/env python3
"""Pruebas rapidas del selector de artefactos sin camara ni MegaPi."""

import copy
import os
import unittest

import cv2
import numpy as np

from vision_core import Detector, cargar_config


AQUI = os.path.dirname(os.path.abspath(__file__))


class DetectorTest(unittest.TestCase):
    def setUp(self):
        self.cfg = copy.deepcopy(cargar_config(os.path.join(AQUI, "config.json")))
        self.detector = Detector(self.cfg)
        self.ancho = self.cfg["camara"].get("ancho", 640)
        self.alto = self.cfg["camara"].get("alto", 360)
        self.cx = self.cfg["geometria"].get("cx_garra", self.ancho // 2)

    def frame(self):
        return np.full((self.alto, self.ancho, 3), 180, dtype=np.uint8)

    def test_auto_prefiere_color_cromatico_sobre_negro(self):
        frame = self.frame()
        cv2.rectangle(frame, (self.cx - 30, int(self.alto * 0.35)),
                             (self.cx + 30, int(self.alto * 0.55)), (0, 0, 0), -1)
        cv2.rectangle(frame, (self.cx - 20, int(self.alto * 0.38)),
                             (self.cx + 20, int(self.alto * 0.52)), (0, 0, 255), -1)

        det, color, _, _ = self.detector.detectar_auto(frame)

        self.assertTrue(det.encontrado)
        self.assertEqual(color, "ROJO")
        self.assertLessEqual(abs(det.ex), 4)

    def test_auto_reconoce_negro_si_no_hay_color(self):
        frame = self.frame()
        cv2.rectangle(frame, (self.cx - 25, int(self.alto * 0.35)),
                             (self.cx + 25, int(self.alto * 0.55)), (0, 0, 0), -1)

        det, color, _, _ = self.detector.detectar_auto(frame)

        self.assertTrue(det.encontrado)
        self.assertEqual(color, "NEGRO")

    def test_dedos_azules_quedan_enmascarados(self):
        frame = self.frame()
        # Coloca color azul en las zonas ignoradas de la garra
        for zona in self.cfg["deteccion"].get("zonas_ignoradas", []):
            x0 = int(float(zona[0]) * self.ancho)
            y0 = int(float(zona[1]) * self.alto)
            x1 = int(float(zona[2]) * self.ancho)
            y1 = int(float(zona[3]) * self.alto)
            cv2.rectangle(frame, (x0, y0), (x1, y1), (255, 0, 0), -1)

        det, _, _ = self.detector.detectar(frame, "AZUL")

        self.assertFalse(det.encontrado)

    def test_suelo_verde_inferior_no_gana_al_artefacto(self):
        frame = self.frame()
        # Franja fina de suelo verde (relacion de aspecto alta > 4.0)
        cv2.rectangle(frame, (0, int(self.alto * 0.88)),
                             (self.ancho, int(self.alto * 0.93)), (0, 150, 0), -1)
        # Artefacto verde compacto a la derecha
        cv2.rectangle(frame, (self.cx + 80, int(self.alto * 0.35)),
                             (self.cx + 140, int(self.alto * 0.55)), (0, 255, 0), -1)

        det, _, _ = self.detector.detectar(frame, "VERDE")

        self.assertTrue(det.encontrado)
        self.assertGreater(det.ex, 50)

    def test_masa_verde_grande_de_pista_se_rechaza(self):
        frame = self.frame()
        # Region compacta dentro de la ROI: sin limite superior pasaria los
        # filtros de area minima, altura y aspecto como si fuera un artefacto.
        cv2.rectangle(frame, (110, 115), (570, 265), (0, 150, 0), -1)

        det, _, _ = self.detector.detectar(frame, "VERDE")

        self.assertFalse(det.encontrado)

    def test_auto_elige_el_artefacto_mas_centrado(self):
        frame = self.frame()
        # Artefacto verde a la izquierda
        cv2.rectangle(frame, (self.cx - 150, int(self.alto * 0.35)),
                             (self.cx - 90, int(self.alto * 0.55)), (0, 255, 0), -1)
        # Artefacto amarillo en el centro
        cv2.rectangle(frame, (self.cx - 25, int(self.alto * 0.35)),
                             (self.cx + 25, int(self.alto * 0.55)), (0, 255, 255), -1)

        det, color, _, _ = self.detector.detectar_auto(frame)

        self.assertEqual(color, "AMARILLO")
        self.assertLessEqual(abs(det.ex), 4)

    def test_fila_asigna_slots_de_izquierda_a_derecha(self):
        frame = self.frame()
        y0, y1 = 105, 190
        cv2.rectangle(frame, (35, y0), (95, y1), (0, 0, 0), -1)
        cv2.rectangle(frame, (185, y0), (245, y1), (0, 255, 0), -1)
        cv2.rectangle(frame, (395, y0), (455, y1), (0, 255, 255), -1)
        cv2.rectangle(frame, (545, y0), (605, y1), (0, 0, 255), -1)

        fila = self.detector.detectar_fila_artefactos(frame)

        self.assertEqual([d.color for d in fila],
                         ["NEGRO", "VERDE", "AMARILLO", "ROJO"])

    def test_destino_exige_cuadro_con_marco_blanco(self):
        frame = np.full((self.alto, self.ancho, 3), 80, dtype=np.uint8)
        cv2.rectangle(frame, (250, 85), (390, 195), (245, 245, 245), -1)
        cv2.rectangle(frame, (285, 110), (355, 170), (0, 0, 220), -1)

        det, _, _ = self.detector.detectar_destino(frame, "ROJO")

        self.assertTrue(det.encontrado)
        self.assertLessEqual(abs(det.ex), 2)

    def test_destino_rechaza_franja_de_pista(self):
        frame = np.full((self.alto, self.ancho, 3), 170, dtype=np.uint8)
        cv2.rectangle(frame, (0, 150), (self.ancho - 1, 178), (0, 0, 220), -1)

        det, _, _ = self.detector.detectar_destino(frame, "ROJO")

        self.assertFalse(det.encontrado)

    def test_destino_prefiere_cuadro_grande_a_ruido_centrado(self):
        frame = np.full((self.alto, self.ancho, 3), 80, dtype=np.uint8)
        # Destino real lateral con marco blanco.
        cv2.rectangle(frame, (405, 75), (575, 195), (245, 245, 245), -1)
        cv2.rectangle(frame, (430, 105), (550, 170), (0, 0, 220), -1)
        # Mancha pequena casi centrada, tambien cuadrada y rodeada de blanco.
        cv2.rectangle(frame, (295, 100), (345, 150), (245, 245, 245), -1)
        cv2.rectangle(frame, (310, 115), (330, 135), (0, 0, 220), -1)

        det, _, _ = self.detector.detectar_destino(frame, "ROJO")

        self.assertTrue(det.encontrado)
        self.assertGreater(det.cx, 450)
        self.assertGreater(det.area, 5000)

    def test_azul_inferior_lateral_no_es_artefacto(self):
        frame = self.frame()
        cv2.rectangle(frame, (550, 260), (625, 350), (255, 0, 0), -1)

        det, _, _ = self.detector.detectar(frame, "AZUL")

        self.assertFalse(det.encontrado)


if __name__ == "__main__":
    unittest.main()
