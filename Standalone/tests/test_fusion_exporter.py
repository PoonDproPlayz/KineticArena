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
    adsk.fusion.TriangleMeshQualityOptions = types.SimpleNamespace(
        HighQualityTriangleMesh=13)
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


class MeshCalculator:
    def __init__(self, mesh):
        self.mesh = mesh

    def setQuality(self, _quality):
        pass

    def calculate(self):
        return self.mesh


class MeshManager:
    def __init__(self, mesh):
        self.mesh = mesh
        self.calls = 0

    def createMeshCalculator(self):
        self.calls += 1
        return MeshCalculator(self.mesh)


class RenderBody:
    name = "Test render body"
    meshManager = MeshManager(types.SimpleNamespace(
        triangleCount=2,
        nodeCoordinates=[Vector(0, 0, 0), Vector(1, 0, 0),
                         Vector(1, 1, 0), Vector(0, 1, 0)],
        nodeIndices=[0, 1, 2, 0, 2, 3],
        normalVectors=[Vector(0, 0, 1), Vector(0, 0, 1),
                       Vector(0, 0, 1), Vector(0, 0, 1)]))


class MeshBody:
    name = "Imported armor panel"
    isClosed = False
    isOriented = True

    def __init__(self, mesh):
        self.displayMesh = mesh


class ClosedMeshBody(MeshBody):
    isClosed = True

    def __init__(self, mesh):
        super().__init__(mesh)
        density = types.SimpleNamespace(
            id="structural_Density", name="Density", value=2700.0,
            units="KilogramPerCubicMeter", hasMultipleValues=False)
        material_properties = types.SimpleNamespace(
            count=1, item=lambda _index: density)
        self.material = types.SimpleNamespace(
            name="Aluminum 7075", materialProperties=material_properties)
        self.parentComponent = types.SimpleNamespace(material=None)


class SurfaceBody(RenderBody):
    isSolid = False


class ExportManager:
    @staticmethod
    def createSTLExportOptions(_body, _filepath):
        return None


class OccurrenceCollection:
    def __init__(self, items=()):
        self._items = list(items)
        self.count = len(self._items)

    def item(self, index):
        return self._items[index]


class Occurrence:
    def __init__(self, name, children=()):
        self.name = name
        self.childOccurrences = OccurrenceCollection(children) if children else None


class BrokenExternalOccurrence:
    name = "Missing linked component:1"

    @property
    def childOccurrences(self):
        raise RuntimeError("The occurrence's referenced component is unavailable")


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

    def test_occurrence_iterator_recurses_through_nested_assembly_tree(self):
        leaf = Occurrence("Wheel:1")
        subassembly = Occurrence("Drive module:1", [leaf])
        top_level = OccurrenceCollection([subassembly, Occurrence("Chassis:1")])

        self.assertEqual(
            [occurrence.name for occurrence in self.exporter._iter_occurrences(top_level)],
            ["Drive module:1", "Wheel:1", "Chassis:1"],
        )

    def test_occurrence_iterator_skips_broken_external_reference_and_continues(self):
        broken = BrokenExternalOccurrence()
        valid = Occurrence("Chassis:1")
        skipped = []
        found = list(self.exporter._iter_occurrences(
            OccurrenceCollection([broken, valid]), skipped))

        self.assertEqual(found, [broken, valid])
        self.assertEqual(len(skipped), 1)
        self.assertIn("Missing linked component:1", skipped[0])
        self.assertIn("referenced component is unavailable", skipped[0])

    def test_body_type_count_separates_solids_from_surface_patches(self):
        solid = types.SimpleNamespace(isSolid=True)
        surface = types.SimpleNamespace(isSolid=False)
        self.assertEqual(
            self.exporter._body_type_counts(OccurrenceCollection([solid, surface])),
            (1, 1))

    def test_render_obj_preserves_shared_vertex_normals_and_writes_uvs(self):
        mesh_path = self._artifact_dir() / "part.obj"
        self.exporter._write_render_mesh_obj(RenderBody(), str(mesh_path), 0.001)
        lines = mesh_path.read_text(encoding="ascii").splitlines()
        self.assertEqual(sum(line.startswith("vn ") for line in lines), 4)
        self.assertEqual(sum(line.startswith("vt ") for line in lines), 6)
        faces = [line for line in lines if line.startswith("f ")]
        self.assertEqual(len(faces), 2)
        # Shared CAD nodes keep the same normal index across both triangles,
        # so the render does not create a lighting seam on their boundary.
        self.assertIn("1/5/1", faces[0])
        self.assertIn("1/8/1", faces[1])
        self.assertIn("3/9/3", faces[1])

    def test_render_obj_reuses_fusion_uvs_per_vertex(self):
        body = RenderBody()
        body.meshManager = MeshManager(types.SimpleNamespace(
            triangleCount=2,
            nodeCoordinates=[Vector(0, 0, 0), Vector(1, 0, 0),
                             Vector(1, 1, 0), Vector(0, 1, 0)],
            nodeIndices=[0, 1, 2, 0, 2, 3],
            normalVectors=[Vector(0, 0, 1)] * 4,
            textureCoordinates=[Vector(0, 0, 0), Vector(1, 0, 0),
                                Vector(1, 1, 0), Vector(0, 1, 0)]))
        mesh_path = self._artifact_dir() / "shared_uv_part.obj"

        self.exporter._write_render_mesh_obj(
            body, str(mesh_path), 0.001, include_uv=True)

        lines = mesh_path.read_text(encoding="ascii").splitlines()
        self.assertEqual(sum(line.startswith("vt ") for line in lines), 4)
        faces = [line for line in lines if line.startswith("f ")]
        self.assertIn("1/1/1", faces[0])
        self.assertIn("1/1/1", faces[1])

    def test_untextured_append_uses_compact_shared_uvs_and_one_tessellation(self):
        body = RenderBody()
        body.isSolid = True
        body.meshManager.calls = 0
        root = self._artifact_dir() / "compact_uv_bundle"
        parts = []

        self.exporter._append_part(
            parts, ExportManager(), body, "Top-level", [], None,
            str(root), 0, "mm", 0.001)

        obj_lines = (root / parts[0]["renderFile"]).read_text(
            encoding="ascii").splitlines()
        self.assertEqual(sum(line.startswith("vt ") for line in obj_lines), 4)
        self.assertEqual(body.meshManager.calls, 1)

    def test_closed_mesh_body_mass_and_mesh_files_share_one_tessellation(self):
        points = [Vector(0, 0, 0), Vector(1, 0, 0),
                  Vector(0, 1, 0), Vector(0, 0, 1)]
        mesh = types.SimpleNamespace(
            triangleCount=4, nodeCoordinates=points,
            nodeIndices=[0, 2, 1, 0, 1, 3, 0, 3, 2, 1, 2, 3],
            normalVectors=[Vector(0, 0, 1)] * 4)
        body = ClosedMeshBody(mesh)
        body.meshManager = MeshManager(mesh)
        root = self._artifact_dir() / "closed_mesh_bundle"
        parts = []

        self.exporter._append_part(
            parts, ExportManager(), body, "Top-level", [], None,
            str(root), 0, "mm", 0.001, mesh_body=True)

        self.assertEqual(body.meshManager.calls, 1)
        self.assertGreater(parts[0]["massKg"], 0)

    def test_mesh_body_exports_stl_and_render_obj_without_brep_mesh_manager(self):
        body = MeshBody(RenderBody.meshManager.mesh)
        root = self._artifact_dir()
        stl_path = root / "mesh_body.stl"
        obj_path = root / "mesh_body.obj"

        self.exporter._write_mesh_stl(body, str(stl_path), 0.001)
        self.exporter._write_render_mesh_obj(body, str(obj_path), 0.001)

        self.assertTrue(self.exporter._valid_binary_stl(str(stl_path)))
        self.assertEqual(stl_path.stat().st_size, 84 + 50 * 2)
        self.assertEqual(sum(line.startswith("f ") for line in
                             obj_path.read_text(encoding="ascii").splitlines()), 2)

    def test_non_solid_surface_body_is_excluded_from_export(self):
        root = self._artifact_dir() / "surface_bundle_exclusion"
        parts = []
        with self.assertRaisesRegex(RuntimeError, "surface bodies are excluded"):
            self.exporter._append_part(
                parts, ExportManager(), SurfaceBody(), "Top-level", [], None,
                str(root), 0, "mm", 0.001)

        self.assertEqual(parts, [])
        self.assertFalse(root.exists())

    def test_closed_mesh_body_derives_mass_and_center_from_material_density(self):
        adsk = sys.modules["adsk"]
        adsk.core.FloatProperty = types.SimpleNamespace(cast=lambda prop: prop)
        points = [Vector(0, 0, 0), Vector(1, 0, 0),
                  Vector(0, 1, 0), Vector(0, 0, 1)]
        mesh = types.SimpleNamespace(
            triangleCount=4, nodeCoordinates=points,
            nodeIndices=[0, 2, 1, 0, 1, 3, 0, 3, 2, 1, 2, 3])
        body = ClosedMeshBody(mesh)

        material, density, mass, volume, center, inertia = self.exporter._body_metadata(
            body, mesh_body=True)

        self.assertEqual(material, "Aluminum 7075")
        self.assertEqual(density, 2700.0)
        self.assertAlmostEqual(volume, 1.0 / 6.0 * 1e-6)
        self.assertAlmostEqual(mass, 2700.0 / 6.0 * 1e-6)
        self.assertEqual(center, [0.0025, 0.0025, 0.0025])
        self.assertIsNone(inertia)

    def test_appearance_maps_are_copied_to_the_bundle(self):
        root = self._artifact_dir()
        source = root / "fusion_color.png"
        source.write_bytes(b"test image bytes")
        appearance = types.SimpleNamespace(
            colorTexture=str(source), normalTexture="", roughnessTexture="")
        output = root / "bundle"
        copied = self.exporter._copy_appearance_textures(appearance, str(output), 4)
        self.assertEqual(copied, {"rgb": "Textures/0004_rgb.png"})
        self.assertEqual((output / copied["rgb"]).read_bytes(), b"test image bytes")

    def test_opaque_material_zero_alpha_uses_body_visibility(self):
        class AppearanceProperties:
            @staticmethod
            def itemById(_name):
                raise KeyError(_name)

        appearance = types.SimpleNamespace(
            name="ABS (White)",
            color=types.SimpleNamespace(red=245, green=245, blue=243, opacity=0),
            appearanceProperties=AppearanceProperties())
        body = types.SimpleNamespace(visibleOpacity=1.0)
        opaque = self.exporter._appearance_metadata(body, appearance=appearance)
        self.assertEqual(opaque["colorRgba"], [245 / 255, 245 / 255, 243 / 255, 1.0])

        class TransparentAppearanceProperties:
            @staticmethod
            def itemById(name):
                if name == "transparent_color":
                    return object()
                raise KeyError(name)

        transparent_appearance = types.SimpleNamespace(
            name="Clear Glass", color=appearance.color,
            appearanceProperties=TransparentAppearanceProperties())
        transparent = self.exporter._appearance_metadata(
            body, appearance=transparent_appearance)
        self.assertEqual(transparent["colorRgba"][3], 0.0)

    @staticmethod
    def _artifact_dir():
        path = Path(__file__).resolve().parents[1] / "build" / "fusion-exporter-test"
        path.mkdir(parents=True, exist_ok=True)
        return path


if __name__ == "__main__":
    unittest.main()
