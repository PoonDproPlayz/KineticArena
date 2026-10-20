"""Build a small Fusion-style bundle and ask native MuJoCo to compile it."""

import json
import math
import re
import struct
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET
from contextlib import contextmanager
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


def chamfered_ring_stl(path, segments=64):
    """Closed annular cylinder with a beveled bore and circular rim edges."""
    import math

    triangles = []

    def point(radius, z, index):
        angle = 2.0 * math.pi * (index % segments) / segments
        return (radius * math.cos(angle), radius * math.sin(angle), z)

    def quad(a, b, c, d):
        triangles.extend(((a, b, c), (a, c, d)))

    profile = [(15, 0), (15, 4), (10, 4), (8, 2), (8, 0), (15, 0)]
    for i in range(segments):
        j = i + 1
        for (r0, z0), (r1, z1) in zip(profile, profile[1:]):
            quad(point(r0, z0, i), point(r0, z0, j),
                 point(r1, z1, j), point(r1, z1, i))

    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("wb") as stream:
        stream.write(bytes(80))
        stream.write(struct.pack("<I", len(triangles)))
        for triangle in triangles:
            a, b, c = triangle
            ab = tuple(b[k] - a[k] for k in range(3))
            ac = tuple(c[k] - a[k] for k in range(3))
            normal = (ab[1] * ac[2] - ab[2] * ac[1],
                      ab[2] * ac[0] - ab[0] * ac[2],
                      ab[0] * ac[1] - ab[1] * ac[0])
            stream.write(struct.pack("<3f", *normal))
            for vertex in triangle:
                stream.write(struct.pack("<3f", *vertex))
            stream.write(struct.pack("<H", 0))


@contextmanager
def smoke_directory(workdir=None):
    if workdir is None:
        with tempfile.TemporaryDirectory(prefix="combatsim_bundle_") as folder:
            yield Path(folder)
    else:
        root = Path(workdir)
        if not root.is_dir():
            raise SystemExit(f"Smoke work directory must already exist: {root}")
        yield root


def main(exe, workdir=None):
    result = subprocess.run([str(exe), "--test-drive-mix"], text=True, capture_output=True)
    print(result.stdout, end="")
    print(result.stderr, end="", file=sys.stderr)
    if result.returncode:
        raise SystemExit(result.returncode)

    result = subprocess.run([str(exe), "--test-motor-torque"], text=True, capture_output=True)
    print(result.stdout, end="")
    print(result.stderr, end="", file=sys.stderr)
    if result.returncode:
        raise SystemExit(result.returncode)

    repo_root = Path(__file__).resolve().parents[2]
    stock_model = repo_root / "Assets" / "CombatSim" / "Models" / "CombatRobot500g.xml"
    result = subprocess.run([str(exe), "--test-wall-impact", str(stock_model)],
                            text=True, capture_output=True)
    print(result.stdout, end="")
    print(result.stderr, end="", file=sys.stderr)
    if result.returncode:
        raise SystemExit(result.returncode)
    assert "Wall impact at 2500 Hz and 100.0 m/s" in result.stdout
    assert "Wall impact stayed inside the arena at both update rates." in result.stdout

    with smoke_directory(workdir) as root:
        entries = [
            ("Frame", 0.35, 0, [-0.1, 0, 0]),
            ("Weapon/Main", 0.10, 1, [0.1, 0, 0]),
            ("Weapon/Blade", 0.02, 1, [0.14, 0, 0]),
            ("Wheels/FL", 0.0075, 2, [0.08, 0.12, 0]),
            ("Wheels/RL", 0.0075, 3, [-0.08, 0.12, 0]),
            ("Wheels/FR", 0.0075, 4, [0.08, -0.12, 0]),
            ("Wheels/RR", 0.0075, 5, [-0.08, -0.12, 0]),
            ("Forks/Contact", 0.01, 6, [-0.12, 0, 0]),
            ("Forks/Pendulum", 0.02, 7, [0.1, 0, 0]),
        ]
        parts = []
        assignments = {}
        for index, (name, mass, group, position) in enumerate(entries):
            relative = f"Parts/{name}/{index:04d}.stl"
            tetra_stl(root / relative)
            part = {"file": relative, "partName": name.split("/")[-1],
                    "bodyName": name, "occurrencePath": name.split("/"),
                    "materialName": "Steel", "massKg": mass,
                    "densityKgPerM3": 7800, "unitToMeters": 0.001,
                    "positionFusionMeters": position,
                    "rotationFusionWxyz": [1, 0, 0, 0],
                    "colorRgba": [0.7, 0.7, 0.75, 1]}
            if name == "Weapon/Main":
                part["centerOfMassLocalMeters"] = [0.005, 0, 0]
                part["inertiaTensorLocalKgM2"] = [4e-6, 5e-6, 6e-6,
                                                   0.5e-6, 0.25e-6, -0.2e-6]
            elif name == "Weapon/Blade":
                part["centerOfMassLocalMeters"] = [0, -0.010, 0]
                part["rotationFusionWxyz"] = [2 ** -0.5, 0, 0, 2 ** -0.5]
                part["inertiaTensorLocalKgM2"] = [1e-6, 2e-6, 3e-6,
                                                   -0.1e-6, 0.2e-6, 0.3e-6]
            parts.append(part)
            assignments[relative] = group
        (root / "manifest.json").write_text(json.dumps({"schemaVersion": 2,
            "designName": "Synthetic bot", "unitToMeters": 0.001,
            "parts": parts}), encoding="utf-8")
        groups = [("Weapon", "Weapon", [0, 1, 0]),
                  ("FL", "Left wheel", [0, 1, 0]),
                  ("RL", "Left wheel", [0, -1, 0]),
                  ("FR", "Right wheel", [0, -1, 0]),
                  ("RR", "Right wheel", [0, 1, 0]),
                  ("Fork contact", "Gravity fork", [0, 1, 0]),
                  ("Fork pendulum", "Gravity fork", [0, 1, 0])]
        pivots = [[0, 0, 0] for _ in groups]
        pivots[5] = [-0.12, 0, 0]
        pivots[6] = [0.1, 0, 0]
        (root / "combatsim_setup.json").write_text(json.dumps({
            "schemaVersion": 1,
            "batteryVoltage": 11.1,
            "assemblies": [{"name": n, "kind": k, "pivotMeters": pivot,
                            "axis": axis,
                            "motor": {"enabled": n in ("FL", "RL"),
                                      "driveGroup": (0 if n in ("FL", "RL") else
                                                     1 if n == "FR" else
                                                     2 if n == "RR" else -1),
                                      "motorCount": 1, "kvRpmPerVolt": 1000.0,
                                      "peakCurrentA": 10.0,
                                      "phaseResistanceOhm": 1.11,
                                      "escCurrentLimitA": 8.0, "gearRatio": 2.0,
                                      "gearboxEfficiency": 0.9}}
                           for pivot, (n, k, axis) in zip(pivots, groups)],
            "partAssignments": assignments}), encoding="utf-8")
        result = subprocess.run([str(exe), "--test-motor-actuator", str(root / "manifest.json")],
                                text=True, capture_output=True)
        print(result.stdout, end="")
        print(result.stderr, end="", file=sys.stderr)
        if result.returncode:
            raise SystemExit(result.returncode)
        assert "Shared motor spin test passed: 2 wheel assemblies" in result.stdout

        result = subprocess.run([str(exe), "--validate-bundle", str(root / "manifest.json")],
                                text=True, capture_output=True)
        print(result.stdout, end="")
        print(result.stderr, end="", file=sys.stderr)
        if result.returncode:
            raise SystemExit(result.returncode)
        assert "CAD wheel drive mapping: left=2 right=2; forward signs L=[1,-1] R=[-1,1]" in result.stdout
        assert "CAD shared motor groups: A=2 wheels" in result.stdout
        fork_test = re.search(
            r"CAD gravity forks: 2; explicit fork-frame pairs: 2; initial fork-frame contacts: (\d+); max passive fork angle after 1.0 s: ([\d.]+) rad",
            result.stdout)
        assert fork_test, "fork contact and passive-motion diagnostics were not reported"
        assert int(fork_test.group(1)) > 0, "fork-to-frame collision was not active at startup"
        assert float(fork_test.group(2)) > 0.02, "gravity did not swing the passive fork module"
        assert float(fork_test.group(2)) <= math.pi / 2 + 0.1, "fork exceeded its default travel limit"
        print("Both-side turn mapping and fitted-axis direction normalization passed")
        print("Passive fork hinge, gravity swing, and main-frame contact passed")
        generated = (root / "combatsim_generated.xml").read_text(encoding="utf-8")
        assert generated.count('<velocity name="cad_assembly_') == 3
        assert generated.count('<motor name="cad_assembly_') == 2
        assert '<motor name="cad_assembly_2_drive" joint="cad_joint_2"/>' in generated
        assert '<motor name="cad_assembly_3_drive" joint="cad_joint_3"/>' in generated
        assert '<velocity name="cad_assembly_1_drive" joint="cad_joint_1"' in generated
        assert 'cad_part_2' in generated  # blade shares the weapon joint
        model = ET.fromstring(generated)
        assert abs(float(model.find("option").attrib["timestep"]) - 0.0004) < 1e-12
        assert model.find(".//motor[@name='cad_assembly_2_drive']") is not None
        assert model.find(".//motor[@name='cad_assembly_3_drive']") is not None
        assert model.find(".//velocity[@name='cad_assembly_1_drive']") is not None
        east_wall = model.find(".//geom[@name='east_wall']")
        assert east_wall is not None
        assert east_wall.attrib["pos"].split()[0] == "1.05"
        assert east_wall.attrib["size"].split()[0] == "0.05"
        assert east_wall.attrib["margin"] == "0.01"
        assert east_wall.attrib["solref"] == "0.001 1"
        assert len(model.findall("contact/pair")) == 2
        for fork_id in (6, 7):
            fork_joint = model.find(f".//joint[@name='cad_joint_{fork_id}']")
            assert fork_joint is not None and fork_joint.attrib["limited"] == "true"
            low, high = (float(value) for value in fork_joint.attrib["range"].split())
            assert abs(low + math.pi / 2) < 1e-9 and abs(high - math.pi / 2) < 1e-9
            assert model.find(f".//velocity[@name='cad_assembly_{fork_id}_drive']") is None
        weapon = model.find(".//body[@name='cad_assembly_1']")
        inertial = weapon.find("inertial")
        combined_com = [float(value) for value in inertial.attrib["pos"].split()]
        combined_mass = float(inertial.attrib["mass"])
        combined_inertia = [float(value) for value in inertial.attrib["fullinertia"].split()]
        expected_com = [0.1125, 0.0, 0.0]
        expected_inertia = [6e-6, 39.75e-6, 42.75e-6,
                            0.6e-6, -0.05e-6, 0.0]
        assert abs(combined_mass - 0.12) < 1e-9
        assert all(abs(actual - expected) < 1e-6
                   for actual, expected in zip(combined_com, expected_com))
        assert all(abs(actual - expected) < 1e-10
                   for actual, expected in zip(combined_inertia, expected_inertia))
        print("Mass-weighted COM and rotated full inertia from both weapon bodies passed")

        # Manual mass edits scale each part's Fusion inertia while preserving
        # its shape (radius of gyration), then recompute the group COM and shift.
        setup_path = root / "combatsim_setup.json"
        setup = json.loads(setup_path.read_text(encoding="utf-8"))
        setup["partOverrides"] = {parts[1]["file"]: {"massKg": 0.2}}
        setup["assemblies"][6]["forkCollidesWithFrame"] = False
        setup_path.write_text(json.dumps(setup), encoding="utf-8")
        result = subprocess.run([str(exe), "--validate-bundle", str(root / "manifest.json")],
                                text=True, capture_output=True)
        print(result.stdout, end="")
        print(result.stderr, end="", file=sys.stderr)
        if result.returncode:
            raise SystemExit(result.returncode)
        fork_toggle = re.search(
            r"CAD gravity forks: 2; explicit fork-frame pairs: (\d+); initial fork-frame contacts: (\d+);",
            result.stdout)
        assert fork_toggle and int(fork_toggle.group(1)) == 1
        assert int(fork_toggle.group(2)) > 0
        generated = (root / "combatsim_generated.xml").read_text(encoding="utf-8")
        model = ET.fromstring(generated)
        assert len(model.findall("contact/pair")) == 1
        assert model.find("contact/pair").attrib["geom1"] == "cad_part_7"
        weapon = model.find(".//body[@name='cad_assembly_1']")
        inertial = weapon.find("inertial")
        combined_com = [float(value) for value in inertial.attrib["pos"].split()]
        combined_mass = float(inertial.attrib["mass"])
        combined_inertia = [float(value) for value in inertial.attrib["fullinertia"].split()]
        expected_com = [0.1090909090909091, 0.0, 0.0]
        expected_inertia = [10e-6, 47.8181818181818e-6, 51.8181818181818e-6,
                            1.1e-6, 0.2e-6, -0.2e-6]
        assert abs(combined_mass - 0.22) < 1e-9
        assert all(abs(actual - expected) < 1e-9
                   for actual, expected in zip(combined_com, expected_com))
        assert all(abs(actual - expected) < 1e-10
                   for actual, expected in zip(combined_inertia, expected_inertia))
        print("Manual mass override inertia scaling and parallel-axis shift passed")

        setup["assemblies"][5]["forkCollidesWithFrame"] = False
        setup_path.write_text(json.dumps(setup), encoding="utf-8")
        result = subprocess.run([str(exe), "--validate-bundle", str(root / "manifest.json")],
                                text=True, capture_output=True)
        print(result.stdout, end="")
        print(result.stderr, end="", file=sys.stderr)
        if result.returncode:
            raise SystemExit(result.returncode)
        assert "CAD gravity forks: 2; explicit fork-frame pairs: 0; initial fork-frame contacts: 0;" in result.stdout
        generated = (root / "combatsim_generated.xml").read_text(encoding="utf-8")
        assert len(ET.fromstring(generated).findall("contact/pair")) == 0
        print("Per-fork main-frame collision toggle passed")

        hole_root = root / "hole_test"
        ring_file = "Parts/Weapon/ring.stl"
        frame_file = "Parts/Frame/frame.stl"
        chamfered_ring_stl(hole_root / ring_file)
        chamfered_ring_stl(hole_root / frame_file)
        hole_manifest = {
            "schemaVersion": 2,
            "designName": "Synthetic hole axis test",
            "unitToMeters": 0.001,
            "parts": [
                {"file": ring_file, "partName": "Ring", "bodyName": "Weapon/Ring",
                 "occurrencePath": ["Weapon", "Ring"], "massKg": 0.01,
                 "unitToMeters": 0.001, "positionFusionMeters": [0, 0, 0],
                 "rotationFusionWxyz": [1, 0, 0, 0]},
                {"file": frame_file, "partName": "Frame", "bodyName": "Frame",
                 "occurrencePath": ["Frame"], "massKg": 0.05,
                 "unitToMeters": 0.001, "positionFusionMeters": [0, 0, 0],
                 "rotationFusionWxyz": [1, 0, 0, 0]},
            ],
        }
        hole_manifest_path = hole_root / "manifest.json"
        hole_manifest_path.write_text(json.dumps(hole_manifest), encoding="utf-8")
        (hole_root / "combatsim_setup.json").write_text(json.dumps({
            "schemaVersion": 1,
            "assemblies": [{"name": "Weapon", "kind": "Weapon",
                            "pivotMeters": [0, 0, 0], "axis": [0, 0, 1]}],
            "partAssignments": {ring_file: 1},
        }), encoding="utf-8")
        result = subprocess.run([str(exe), "--test-hole-axis", str(hole_manifest_path)],
                                text=True, capture_output=True)
        print(result.stdout, end="")
        print(result.stderr, end="", file=sys.stderr)
        if result.returncode:
            raise SystemExit(result.returncode)


if __name__ == "__main__":
    main(Path(sys.argv[1]), Path(sys.argv[2]) if len(sys.argv) > 2 else None)
