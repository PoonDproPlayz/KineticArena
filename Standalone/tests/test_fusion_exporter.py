"""Unit tests for Fusion physical-property export without launching Fusion."""

import importlib.util
import math
import sys
import types
import unittest
from pathlib import Path


def load_exporter():
    adsk = types.ModuleType("adsk")
    adsk.core = types.ModuleType("adsk.core")
    adsk.fusion = types.ModuleType("adsk.fusion")
    sys.modules["adsk"] = adsk
    sys.modules["adsk.core"] = adsk.core
    sys.modules["adsk.fusion"] = adsk.fusion
    path = Path(__file__).resolve().parents[2] / "Fusion360Exporter" / "CombatSimExport.py"
    spec = importlib.util.spec_from_file_location("combatsim_fusion_exporter", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class Vector:
    def __init__(self, x, y, z):
        self.x, self.y, self.z = x, y, z


class Properties:
    mass = 0.125
    density = 7.8e-3  # kg/cm^3
    centerOfMass = Vector(1.0, -2.0, 3.0)  # cm

    def getPrincipalMomentsOfInertia(self):
        return True, 1.0, 3.0, 5.0  # kg*cm^2

    def getPrincipalAxes(self):
        diagonal = math.sqrt(0.5)
        return (True, Vector(diagonal, diagonal, 0),
                Vector(-diagonal, diagonal, 0), Vector(0, 0, 1))


class Body:
    material = None
    parentComponent = types.SimpleNamespace(material=None)
    physicalProperties = Properties()
    volume = 10.0  # cm^3


class FusionExporterTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.exporter = load_exporter()

    def test_principal_properties_become_centroidal_tensor_in_si_units(self):
        material, density, mass, volume, center, inertia = self.exporter._body_metadata(Body())
        self.assertEqual(material, "Unknown / Custom")
        self.assertAlmostEqual(density, 7800.0)
        self.assertAlmostEqual(mass, 0.125)
        self.assertAlmostEqual(volume, 1e-5)
        self.assertEqual(center, [0.01, -0.02, 0.03])
        expected = [2e-4, 2e-4, 5e-4, -1e-4, 0.0, 0.0]
        for actual, wanted in zip(inertia, expected):
            self.assertAlmostEqual(actual, wanted, places=12)


if __name__ == "__main__":
    unittest.main()
