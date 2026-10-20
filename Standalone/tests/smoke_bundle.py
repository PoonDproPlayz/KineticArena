"""Build a small Fusion-style bundle and ask native MuJoCo to compile it."""

import json
import struct
import subprocess
import sys
import tempfile
from pathlib import Path


def tetra_stl(path):
    vertices = [(0, 0, 0), (70, 0, 0), (0, 70, 0), (0, 0, 70)]
    faces = [(0, 2, 1), (0, 1, 3), (0, 3, 2), (1, 2, 3)]
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("wb") as stream:
        stream.write(bytes(80))
        stream.write(struct.pack("<I", len(faces)))
        for face in faces:
            stream.write(struct.pack("<3f", 0, 0, 1))
            for index in face:
                stream.write(struct.pack("<3f", *vertices[index]))
            stream.write(struct.pack("<H", 0))


def main(exe):
    with tempfile.TemporaryDirectory(prefix="combatsim_bundle_") as folder:
        root = Path(folder)
        entries = [
            ("Frame", 0.35, 0, [-0.1, 0, 0]),
            ("Weapon/Main", 0.10, 1, [0.1, 0, 0]),
            ("Weapon/Blade", 0.02, 1, [0.14, 0, 0]),
            ("Wheels/FL", 0.0075, 2, [0.08, 0.12, 0]),
            ("Wheels/RL", 0.0075, 3, [-0.08, 0.12, 0]),
            ("Wheels/FR", 0.0075, 4, [0.08, -0.12, 0]),
            ("Wheels/RR", 0.0075, 5, [-0.08, -0.12, 0]),
        ]
        parts = []
        assignments = {}
        for index, (name, mass, group, position) in enumerate(entries):
            relative = f"Parts/{name}/{index:04d}.stl"
            tetra_stl(root / relative)
            parts.append({"file": relative, "partName": name.split("/")[-1],
                          "bodyName": name, "occurrencePath": name.split("/"),
                          "materialName": "Steel", "massKg": mass,
                          "densityKgPerM3": 7800, "unitToMeters": 0.001,
                          "positionFusionMeters": position,
                          "rotationFusionWxyz": [1, 0, 0, 0],
                          "colorRgba": [0.7, 0.7, 0.75, 1]})
            assignments[relative] = group
        (root / "manifest.json").write_text(json.dumps({"schemaVersion": 2,
            "designName": "Synthetic bot", "unitToMeters": 0.001,
            "parts": parts}), encoding="utf-8")
        groups = [("Weapon", "Weapon"), ("FL", "Left wheel"),
                  ("RL", "Left wheel"), ("FR", "Right wheel"),
                  ("RR", "Right wheel")]
        (root / "combatsim_setup.json").write_text(json.dumps({
            "schemaVersion": 1,
            "assemblies": [{"name": n, "kind": k, "pivotMeters": [0, 0, 0],
                            "axis": [0, 1, 0]} for n, k in groups],
            "partAssignments": assignments}), encoding="utf-8")
        result = subprocess.run([str(exe), "--validate-bundle", str(root / "manifest.json")],
                                text=True, capture_output=True)
        print(result.stdout, end="")
        print(result.stderr, end="", file=sys.stderr)
        if result.returncode:
            raise SystemExit(result.returncode)
        generated = (root / "combatsim_generated.xml").read_text(encoding="utf-8")
        assert generated.count('<velocity name="cad_assembly_') == 5
        assert 'cad_part_2' in generated  # blade shares the weapon joint


if __name__ == "__main__":
    main(Path(sys.argv[1]))
