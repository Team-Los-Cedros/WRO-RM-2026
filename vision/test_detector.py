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
        cfg = copy.deepcopy(cargar_config(os.path.join(AQUI, "config.json")))
        self.detector = Detector(cfg)

    @staticmethod
    def frame():
        return np.full((240, 320, 3), 180, dtype=np.uint8)

    def test_auto_prefiere_color_cromatico_sobre_negro(self):
        frame = self.frame()
        cv2.rectangle(frame, (130, 55), (190, 115), (0, 0, 0), -1)
        cv2.rectangle(frame, (140, 65), (180, 105), (0, 0, 255), -1)

        det, color, _, _ = self.detector.detectar_auto(frame)

        self.assertTrue(det.encontrado)
        self.assertEqual(color, "ROJO")
        self.assertLessEqual(abs(det.ex), 2)

    def test_auto_reconoce_negro_si_no_hay_color(self):
        frame = self.frame()
        cv2.rectangle(frame, (135, 55), (185, 115), (0, 0, 0), -1)

        det, color, _, _ = self.detector.detectar_auto(frame)

        self.assertTrue(det.encontrado)
        self.assertEqual(color, "NEGRO")

    def test_dedos_azules_quedan_enmascarados(self):
        frame = self.frame()
        cv2.rectangle(frame, (60, 175), (105, 235), (255, 0, 0), -1)
        cv2.rectangle(frame, (205, 175), (255, 235), (255, 0, 0), -1)

        det, _, _ = self.detector.detectar(frame, "AZUL")

        self.assertFalse(det.encontrado)

    def test_suelo_verde_inferior_no_gana_al_artefacto(self):
        frame = self.frame()
        cv2.rectangle(frame, (80, 180), (240, 230), (0, 150, 0), -1)
        cv2.rectangle(frame, (205, 35), (270, 95), (0, 255, 0), -1)

        det, _, _ = self.detector.detectar(frame, "VERDE")

        self.assertTrue(det.encontrado)
        self.assertGreater(det.ex, 60)
        self.assertLess(det.ey, 110)

    def test_auto_elige_el_artefacto_mas_centrado(self):
        frame = self.frame()
        cv2.rectangle(frame, (55, 45), (105, 100), (0, 255, 0), -1)
        cv2.rectangle(frame, (135, 50), (185, 105), (0, 255, 255), -1)

        det, color, _, _ = self.detector.detectar_auto(frame)

        self.assertEqual(color, "AMARILLO")
        self.assertLessEqual(abs(det.ex), 2)


if __name__ == "__main__":
    unittest.main()
