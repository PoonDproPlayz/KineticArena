"""Build a small Fusion-style bundle and ask native MuJoCo to compile it."""

import json
import math
import re
import struct
import zlib
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


def tetra_obj(path):
    """Small OBJ with vertex normals and UVs, like the new Fusion render mesh."""
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("""# test OBJ with UVs and smooth normals
v 0 0 0
v 70 0 0
v 0 70 0
v 0 0 70
vt 0 0
vt 1 0
vt 0 1
vt 1 1
vn 0 0 1
vn 0 1 0
vn 1 0 0
vn 0.57735 0.57735 0.57735
f 1/1/1 3/3/3 2/2/2
f 1/1/1 2/2/2 4/4/4
f 1/1/1 4/4/4 3/3/3
f 2/2/2 3/3/3 4/4/4
""", encoding="ascii")


def tiny_png(path):
    path.parent.mkdir(parents=True, exist_ok=True)
    def chunk(kind, payload):
        content = kind + payload
        return struct.pack(">I", len(payload)) + content + struct.pack(">I", zlib.crc32(content))
    image = bytes((0, 255, 0, 0, 255, 0, 255, 0, 255,
                   0, 0, 0, 255, 255, 255, 255, 255, 255))
    png = (b"\x89PNG\r\n\x1a\n" +
           chunk(b"IHDR", struct.pack(">IIBBBBB", 2, 2, 8, 6, 0, 0, 0)) +
           chunk(b"IDAT", zlib.compress(image)) +
           chunk(b"IEND", b""))
    path.write_bytes(png)


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


def floor_scrape_pairs(model):
    return [pair for pair in model.findall("contact/pair")
            if "floor" in (pair.attrib.get("geom1"), pair.attrib.get("geom2"))]


def robot_internal_pairs(model):
    return [pair for pair in model.findall("contact/pair")
            if "floor" not in (pair.attrib.get("geom1"), pair.attrib.get("geom2"))]


def has_pair(model, geom_a, geom_b):
    return any({pair.attrib.get("geom1"), pair.attrib.get("geom2")} == {geom_a, geom_b}
               for pair in model.findall("contact/pair"))


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
    result = subprocess.run([str(exe), "--test-input-mixer"], text=True, capture_output=True)
    print(result.stdout, end="")
    print(result.stderr, end="", file=sys.stderr)
    if result.returncode:
        raise SystemExit(result.returncode)

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
    result = subprocess.run([str(exe), "--test-arena-spawns", str(stock_model)],
                            text=True, capture_output=True)
    print(result.stdout, end="")
    print(result.stderr, end="", file=sys.stderr)
    if result.returncode:
        raise SystemExit(result.returncode)
    assert "matte laminate floor" in result.stdout
    stock_xml = ET.parse(stock_model).getroot()
    stock_gravity = [float(value) for value in stock_xml.find("option").attrib["gravity"].split()]
    assert stock_gravity == [0.0, 0.0, -9.80665], "stock model must use standard Earth gravity"
    for material_name in ("bot_painted_alloy", "bot_tire_rubber",
                          "bot_machined_steel", "bot_tool_steel"):
        assert stock_xml.find(f".//material[@name='{material_name}']") is not None
    assert stock_xml.find(".//geom[@name='chassis']").attrib["material"] == "bot_painted_alloy"
    assert stock_xml.find(".//body[@name='front_left_wheel']/geom").attrib["material"] == "bot_tire_rubber"
    assert stock_xml.find(".//geom[@name='drum_core']").attrib["material"] == "bot_machined_steel"
    result = subprocess.run([str(exe), "--test-wall-impact", str(stock_model)],
                            text=True, capture_output=True)
    print(result.stdout, end="")
    print(result.stderr, end="", file=sys.stderr)
    if result.returncode:
        raise SystemExit(result.returncode)
    assert "Wall impact at 2500 Hz and 100.0 m/s" in result.stdout
    assert "Wall, roof, and diagonal corner impacts stayed inside the hidden catch volumes" in result.stdout
    result = subprocess.run([str(exe), "--test-transform-gizmo", str(stock_model)],
                            text=True, capture_output=True)
    print(result.stdout, end="")
    print(result.stderr, end="", file=sys.stderr)
    if result.returncode:
        raise SystemExit(result.returncode)
    assert "X/Y/Z arrows and rotation rings render and pick" in result.stdout

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
            part_data = {"file": relative, "partName": name.split("/")[-1],
                         "bodyName": name, "occurrencePath": name.split("/"),
                         "materialName": "Steel", "massKg": mass,
                         "densityKgPerM3": 7800, "unitToMeters": 0.001,
                         "positionFusionMeters": position,
                         "rotationFusionWxyz": [1, 0, 0, 0],
                         "colorRgba": [0.7, 0.7, 0.75, 1]}
            if index == 0:
                # Fusion versions can emit an opaque ABS material with alpha
                # zero; physical solids should remain visible in the preview.
                part_data["materialName"] = "UHMW, White"
                part_data["appearanceName"] = "ABS (White)"
                part_data["colorRgba"] = [0.96, 0.96, 0.95, 0]
                render_file = Path(relative).with_suffix(".obj").as_posix()
                texture_file = "Textures/frame_color.png"
                tetra_obj(root / render_file)
                tiny_png(root / texture_file)
                part_data["renderFile"] = render_file
                part_data["textures"] = {"rgb": texture_file}
            elif index == 8:
                # Preserve alpha zero for an explicitly transparent appearance.
                part_data["materialName"] = "Clear polycarbonate"
                part_data["appearanceName"] = "Clear Polycarbonate"
                part_data["colorRgba"] = [0.8, 0.9, 1.0, 0]
            part = part_data
            if name == "Weapon/Main":
                part["centerOfMassLocalMeters"] = [0.005, 0, 0]
                part["inertiaTensorLocalKgM2"] = [4e-6, 5e-6, 6e-6,
                                                   0.5e-6, 0.25e-6, -0.2e-6]
            if index == 0:
                part["roughness"] = 0.25
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
            "showAxisMarkers": False,
            # Existing setups may have saved the bad Fusion alpha=0 value.
            "partOverrides": {
                parts[0]["file"]: {"colorRgba": [0.96, 0.96, 0.95, 0]}
            },
            "assemblies": [{"name": n, "kind": k, "pivotMeters": pivot,
                            "axis": axis,
                            "motor": {"enabled": n in ("FL", "RL", "Weapon"),
                                      "driveGroup": (0 if n in ("FL", "RL") else
                                                     1 if n == "FR" else
                                                     2 if n == "RR" else -1),
                                      "motorCount": 1, "kvRpmPerVolt": 1000.0,
                                      "peakCurrentA": 10.0,
                                      "phaseResistanceOhm": 1.11,
                                      "escCurrentLimitA": 8.0, "gearRatio": "3:2",
                                      "gearboxEfficiency": 0.9}}
                           for pivot, (n, k, axis) in zip(pivots, groups)],
            "partAssignments": assignments}), encoding="utf-8")
        opponent_root = root / "opponent"
        tetra_stl(opponent_root / "Parts" / "chassis.stl")
        tetra_stl(opponent_root / "Parts" / "weapon.stl")
        tetra_stl(opponent_root / "Parts" / "wheel.stl")
        tetra_stl(opponent_root / "Parts" / "wheel_right.stl")
        tetra_stl(opponent_root / "Parts" / "fork.stl")
        opponent_parts = [
            {"file": "Parts/chassis.stl", "partName": "Chassis",
             "bodyName": "Chassis", "occurrencePath": ["Chassis"],
             "massKg": 0.18, "unitToMeters": 0.001,
             "positionFusionMeters": [0, 0, 0], "rotationFusionWxyz": [1, 0, 0, 0]},
            {"file": "Parts/weapon.stl", "partName": "Weapon",
             "bodyName": "Weapon", "occurrencePath": ["Weapon"],
             "massKg": 0.05, "unitToMeters": 0.001,
             "positionFusionMeters": [0.15, 0, 0.01], "rotationFusionWxyz": [1, 0, 0, 0]},
            {"file": "Parts/wheel.stl", "partName": "Wheel",
             "bodyName": "Wheel", "occurrencePath": ["Wheel"],
             "massKg": 0.04, "unitToMeters": 0.001,
             "positionFusionMeters": [-0.15, 0, 0], "rotationFusionWxyz": [1, 0, 0, 0]},
            {"file": "Parts/fork.stl", "partName": "Fork",
             "bodyName": "Fork", "occurrencePath": ["Fork"],
             "massKg": 0.03, "unitToMeters": 0.001,
             "positionFusionMeters": [0, 0.15, 0], "rotationFusionWxyz": [1, 0, 0, 0]},
            {"file": "Parts/wheel_right.stl", "partName": "Right wheel",
             "bodyName": "Right wheel", "occurrencePath": ["Right wheel"],
             "massKg": 0.04, "unitToMeters": 0.001,
             "positionFusionMeters": [0.15, 0, 0], "rotationFusionWxyz": [1, 0, 0, 0]},
        ]
        opponent_manifest = opponent_root / "manifest.json"
        opponent_manifest.write_text(json.dumps({"schemaVersion": 2,
            "designName": "Synthetic opponent", "unitToMeters": 0.001,
            "parts": opponent_parts}), encoding="utf-8")
        (opponent_root / "combatsim_setup.json").write_text(json.dumps({
            "schemaVersion": 1,
            "batteryVoltage": 11.1,
            "assemblies": [
                {"name": "Weapon", "kind": "Weapon", "pivotMeters": [0.15, 0, 0.01],
                 "axis": [0, 1, 0], "motor": {"enabled": True, "motorCount": 1,
                     "kvRpmPerVolt": 1000.0, "peakCurrentA": 10.0,
                     "phaseResistanceOhm": 0.04, "gearRatio": "1:1",
                     "gearboxEfficiency": 0.9}},
                {"name": "Drive wheel left", "kind": "Left wheel", "pivotMeters": [-0.15, 0, 0],
                 "axis": [1, 0, 0], "motor": {"enabled": True, "driveGroup": 0,
                     "motorCount": 1, "kvRpmPerVolt": 1000.0, "peakCurrentA": 10.0,
                     "phaseResistanceOhm": 0.04, "gearRatio": "1:1",
                     "gearboxEfficiency": 0.9}},
                {"name": "Drive wheel right", "kind": "Right wheel", "pivotMeters": [0.15, 0, 0],
                 "axis": [1, 0, 0], "motor": {"enabled": True, "driveGroup": 1,
                     "motorCount": 1, "kvRpmPerVolt": 1000.0, "peakCurrentA": 10.0,
                     "phaseResistanceOhm": 0.04, "gearRatio": "1:1",
                     "gearboxEfficiency": 0.9}},
                {"name": "Front fork", "kind": "Gravity fork", "pivotMeters": [0, 0.15, 0],
                 "axis": [0, 1, 0], "forkRangeDegrees": [-65, 70],
                 "forkCollidesWithFrame": True},
            ],
            "partAssignments": {"Parts/chassis.stl": 0, "Parts/weapon.stl": 1,
                                 "Parts/wheel.stl": 2, "Parts/wheel_right.stl": 3,
                                 "Parts/fork.stl": 4},
        }), encoding="utf-8")
        result = subprocess.run([str(exe), "--test-opponent-bundle",
                                 str(root / "manifest.json"), str(opponent_manifest)],
                                text=True, capture_output=True)
        print(result.stdout, end="")
        print(result.stderr, end="", file=sys.stderr)
        if result.returncode:
            raise SystemExit(result.returncode)
        assert "0.34 kg total mass" in result.stdout
        assert "opposite spawn" in result.stdout
        assert "guest-only axis visibility" in result.stdout
        assert "player collision" in result.stdout
        opponent_model = ET.parse(root / "combatsim_generated.xml").getroot()
        opponent_surface = opponent_model.find(".//material[@name='opponent_surface_0']")
        assert opponent_surface is not None
        assert opponent_model.find(".//geom[@name='opponent_part_0']").attrib["material"] == "opponent_surface_0"
        assert has_pair(opponent_model, "floor", "opponent_part_0")
        assert has_pair(opponent_model, "floor", "opponent_part_3")
        for part_name in ("opponent_part_0", "opponent_part_3"):
            pair = next(pair for pair in floor_scrape_pairs(opponent_model)
                        if part_name in (pair.attrib.get("geom1"), pair.attrib.get("geom2")))
            assert pair.attrib["solref"] == "0.02 2"
        result = subprocess.run([str(exe), "--test-renderer-resume",
                                 str(root / "manifest.json"), str(opponent_manifest)],
                                text=True, capture_output=True)
        print(result.stdout, end="")
        print(result.stderr, end="", file=sys.stderr)
        if result.returncode:
            raise SystemExit(result.returncode)
        assert "Renderer restart restored player and opponent CAD bundles" in result.stdout
        opponent_quat = [float(value) for value in
                         opponent_model.find(".//body[@name='Target_Robot']").attrib["quat"].split()]
        assert abs(opponent_quat[0] - 2 ** -0.5) < 1e-6
        assert abs(opponent_quat[3] - 2 ** -0.5) < 1e-6
        result = subprocess.run([str(exe), "--test-motor-actuator", str(root / "manifest.json")],
                                text=True, capture_output=True)
        print(result.stdout, end="")
        print(result.stderr, end="", file=sys.stderr)
        if result.returncode:
            raise SystemExit(result.returncode)
        assert "Shared motor spin test passed: 2 wheel assemblies" in result.stdout
        assert "Manual speed override passed: 4000 RPM target reached" in result.stdout
        assert "7400 RPM no-load" in result.stdout

        setup_path = root / "combatsim_setup.json"
        setup = json.loads(setup_path.read_text(encoding="utf-8"))
        result = subprocess.run([str(exe), "--test-drive-layout", str(root / "manifest.json")],
                                text=True, capture_output=True)
        print(result.stdout, end="")
        print(result.stderr, end="", file=sys.stderr)
        if result.returncode:
            raise SystemExit(result.returncode)
        assert "2WD front/rear axle selection and return to 4WD passed" in result.stdout
        saved_layout = json.loads(setup_path.read_text(encoding="utf-8"))
        assert saved_layout["driveLayout"] == "4WD"
        assert saved_layout["twoWdAxle"] == "rear"
        assert all(assembly.get("wheelPosition") in ("front", "rear")
                   for assembly in saved_layout["assemblies"]
                   if assembly["kind"] in ("Left wheel", "Right wheel"))

        # Brushed motors often publish speed at a voltage instead of Kv and may
        # omit resistance/controller details. Confirm both speed data and those
        # missing optional values load through the real bundle path.
        brushed_setup = json.loads(json.dumps(setup))
        brushed_setup["driveLayout"] = "4WD"
        for assembly in brushed_setup["assemblies"]:
            if assembly["name"] in ("FL", "RL"):
                motor = assembly["motor"]
                motor["motorType"] = "brushed"
                motor["speedInput"] = "ratedRpmAtVoltage"
                motor["ratedRpm"] = 10000.0
                motor["ratedVoltage"] = 12.0
                motor.pop("phaseResistanceOhm", None)
                motor.pop("armatureResistanceOhm", None)
                motor.pop("escCurrentLimitA", None)
        setup_path.write_text(json.dumps(brushed_setup), encoding="utf-8")
        result = subprocess.run([str(exe), "--test-motor-actuator", str(root / "manifest.json")],
                                text=True, capture_output=True)
        print(result.stdout, end="")
        print(result.stderr, end="", file=sys.stderr)
        setup_path.write_text(json.dumps(setup), encoding="utf-8")
        if result.returncode:
            raise SystemExit(result.returncode)
        assert "6167 RPM no-load" in result.stdout
        assert "Manual speed override passed: 4000 RPM target reached" in result.stdout

        result = subprocess.run([str(exe), "--test-weapon-motor", str(root / "manifest.json")],
                                text=True, capture_output=True)
        print(result.stdout, end="")
        print(result.stderr, end="", file=sys.stderr)
        if result.returncode:
            raise SystemExit(result.returncode)
        assert "Weapon torque spin-up passed" in result.stdout
        assert "estimated 0-to-90% time" in result.stdout
        assert "7400 RPM no-load" in result.stdout

        # Legacy setup files stored a numeric motor-speed reduction. It must
        # still load with the same input:output value and no-load estimate.
        setup_path = root / "combatsim_setup.json"
        setup = json.loads(setup_path.read_text(encoding="utf-8"))
        for assembly in setup["assemblies"]:
            if "motor" in assembly:
                assembly["motor"]["gearRatio"] = 1.5
        setup_path.write_text(json.dumps(setup), encoding="utf-8")
        result = subprocess.run([str(exe), "--test-weapon-motor", str(root / "manifest.json")],
                                text=True, capture_output=True)
        print(result.stdout, end="")
        print(result.stderr, end="", file=sys.stderr)
        if result.returncode:
            raise SystemExit(result.returncode)
        assert "7400 RPM no-load" in result.stdout
        print("Legacy numeric gear ratio loaded as 3:2")

        result = subprocess.run([str(exe), "--validate-bundle", str(root / "manifest.json")],
                                text=True, capture_output=True)
        print(result.stdout, end="")
        print(result.stderr, end="", file=sys.stderr)
        if result.returncode:
            raise SystemExit(result.returncode)
        assert "Assembly axis markers: hidden" in result.stdout
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
        result = subprocess.run([str(exe), "--test-arena-spawns",
                                 str(root / "combatsim_generated.xml")],
                                text=True, capture_output=True)
        print(result.stdout, end="")
        print(result.stderr, end="", file=sys.stderr)
        if result.returncode:
            raise SystemExit(result.returncode)
        assert "safe center-line and diagonal spawns, and selected-layout reset" in result.stdout
        generated = (root / "combatsim_generated.xml").read_text(encoding="utf-8")
        assert generated.count('<velocity name="cad_assembly_') == 4
        assert generated.count('<motor name="cad_assembly_') == 3
        assert '<motor name="cad_assembly_1_drive" joint="cad_joint_1"/>' in generated
        assert '<motor name="cad_assembly_2_drive" joint="cad_joint_2"/>' in generated
        assert '<motor name="cad_assembly_3_drive" joint="cad_joint_3"/>' in generated
        assert '<velocity name="cad_assembly_2_manual_drive" joint="cad_joint_2"' in generated
        assert '<velocity name="cad_assembly_3_manual_drive" joint="cad_joint_3"' in generated
        assert 'cad_part_2' in generated  # blade shares the weapon joint
        model = ET.fromstring(generated)
        assert abs(float(model.find("option").attrib["timestep"]) - 0.0004) < 1e-12
        generated_gravity = [float(value) for value in model.find("option").attrib["gravity"].split()]
        assert generated_gravity == [0.0, 0.0, -9.80665], "Fusion model must use standard Earth gravity"
        cad_surface = model.find(".//material[@name='cad_surface_0']")
        assert cad_surface is not None
        rgb_layer = cad_surface.find("./layer[@role='rgb']")
        if rgb_layer is not None:
            assert rgb_layer.attrib["texture"] == "cad_texture_0_rgb"
        else:
            assert cad_surface.attrib["texture"] == "cad_texture_0_rgb"
        assert model.find(".//texture[@name='cad_texture_0_rgb']").attrib["file"] == "Textures/frame_color.png"
        assert model.find(".//mesh[@name='cad_mesh_0']").attrib["file"].endswith("0000.obj")
        assert model.find(".//mesh[@name='cad_mesh_0']").attrib["smoothnormal"] == "true"
        assert model.find(".//mesh[@name='cad_mesh_1']").attrib["smoothnormal"] == "true"
        assert abs(float(cad_surface.attrib["specular"]) - 0.29325) < 1e-9
        assert abs(float(cad_surface.attrib["shininess"]) - 0.675) < 1e-9
        assert model.find(".//geom[@name='cad_part_0']").attrib["material"] == "cad_surface_0"
        preview_model = ET.parse(root / "combatsim_preview.xml").getroot()
        opaque_preview_surface = preview_model.find(
            ".//material[@name='preview_surface_0']")
        assert opaque_preview_surface is not None
        assert abs(float(opaque_preview_surface.attrib["rgba"].split()[3]) - 1.0) < 1e-9
        transparent_surface = preview_model.find(
            ".//material[@name='preview_surface_8']")
        assert transparent_surface is not None
        assert float(transparent_surface.attrib["rgba"].split()[3]) == 0.0
        floor = model.find(".//geom[@name='floor']")
        assert floor is not None and floor.attrib["type"] == "plane"
        assert floor.attrib["rgba"] == "0 0 0 0"
        floor_visual = model.find(".//geom[@name='floor_visual']")
        assert floor_visual is not None and floor_visual.attrib["material"] == "arena_floor_laminate"
        assert floor_visual.attrib["type"] == "box" and floor_visual.attrib["pos"] == "0 0 -0.001"
        assert floor_visual.attrib["size"] == "1 1 0.001"
        assert floor_visual.attrib["contype"] == "0" and floor_visual.attrib["conaffinity"] == "0"
        floor_material = model.find(".//material[@name='arena_floor_laminate']")
        assert floor_material is not None and float(floor_material.attrib["rgba"].split()[0]) > 0.75
        key_light = model.find(".//light[@name='arena_key_light']")
        fill_light = model.find(".//light[@name='arena_fill_light']")
        assert key_light is not None and key_light.attrib["castshadow"] == "false"
        assert fill_light is not None and fill_light.attrib["castshadow"] == "false"
        assert model.find(".//motor[@name='cad_assembly_2_drive']") is not None
        assert model.find(".//motor[@name='cad_assembly_3_drive']") is not None
        assert model.find(".//motor[@name='cad_assembly_1_drive']") is not None
        east_wall = model.find(".//geom[@name='east_wall']")
        assert east_wall is not None
        assert east_wall.attrib["pos"].split()[0] == "1.30"
        assert east_wall.attrib["size"].split()[0] == "0.30"
        assert float(east_wall.attrib["size"].split()[1]) >= 1.619
        assert east_wall.attrib["margin"] == "0.01"
        assert east_wall.attrib["solref"] == "0.001 1"
        assert float(model.find(".//geom[@name='north_wall']").attrib["size"].split()[0]) >= 1.619
        assert float(model.find(".//geom[@name='roof']").attrib["size"].split()[0]) >= 1.619
        east_panel = model.find(".//geom[@name='east_wall_acrylic']")
        assert east_panel is not None
        assert east_panel.attrib["size"].split()[0] == "0.004"
        assert east_panel.attrib["contype"] == east_panel.attrib["conaffinity"] == "0"
        assert len(floor_scrape_pairs(model)) == 3
        assert len(robot_internal_pairs(model)) == 2
        assert all(pair.attrib["solref"] == "0.003 1"
                   for pair in robot_internal_pairs(model))
        assert has_pair(model, "floor", "cad_part_0")
        frame_floor_pair = next(pair for pair in floor_scrape_pairs(model)
                                if "cad_part_0" in (pair.attrib.get("geom1"), pair.attrib.get("geom2")))
        assert frame_floor_pair.attrib["solref"] == "0.02 2"
        for fork_id in (6, 7):
            fork_joint = model.find(f".//joint[@name='cad_joint_{fork_id}']")
            assert fork_joint is not None and fork_joint.attrib["limited"] == "true"
            low, high = (float(value) for value in fork_joint.attrib["range"].split())
            assert abs(low + math.pi / 2) < 1e-9 and abs(high - math.pi / 2) < 1e-9
            assert model.find(f".//velocity[@name='cad_assembly_{fork_id}_drive']") is None

        # Fixed CAD forks stay collision-enabled while their floor response is
        # damped by a floor-specific pair; other collisions keep default response.
        setup_path = root / "combatsim_setup.json"
        setup = json.loads(setup_path.read_text(encoding="utf-8"))
        fixed_fork_path = "Parts/Forks/Contact/0007.stl"
        setup["partAssignments"][fixed_fork_path] = 0
        setup_path.write_text(json.dumps(setup), encoding="utf-8")
        result = subprocess.run([str(exe), "--validate-bundle", str(root / "manifest.json")],
                                text=True, capture_output=True)
        print(result.stdout, end="")
        print(result.stderr, end="", file=sys.stderr)
        if result.returncode:
            raise SystemExit(result.returncode)
        fixed_model = ET.parse(root / "combatsim_generated.xml").getroot()
        assert has_pair(fixed_model, "floor", "cad_part_7")
        fixed_fork_pair = next(pair for pair in floor_scrape_pairs(fixed_model)
                               if "cad_part_7" in (pair.attrib.get("geom1"), pair.attrib.get("geom2")))
        assert fixed_fork_pair.attrib["solref"] == "0.02 2"
        fixed_fork = fixed_model.find(".//geom[@name='cad_part_7']")
        assert fixed_fork is not None
        assert "priority" not in fixed_fork.attrib and "solref" not in fixed_fork.attrib
        setup["partAssignments"][fixed_fork_path] = 6
        setup_path.write_text(json.dumps(setup), encoding="utf-8")
        print("Chassis and fixed fork floor-contact damping passed")

        setup["partAssignments"][fixed_fork_path] = 0
        setup["partOverrides"] = {fixed_fork_path: {"collisionEnabled": False}}
        setup_path.write_text(json.dumps(setup), encoding="utf-8")
        result = subprocess.run([str(exe), "--validate-bundle", str(root / "manifest.json")],
                                text=True, capture_output=True)
        print(result.stdout, end="")
        print(result.stderr, end="", file=sys.stderr)
        if result.returncode:
            raise SystemExit(result.returncode)
        disabled_model = ET.parse(root / "combatsim_generated.xml").getroot()
        disabled_fork = disabled_model.find(".//geom[@name='cad_part_7']")
        assert disabled_fork is not None and disabled_fork.attrib["contype"] == "0"
        assert not has_pair(disabled_model, "floor", "cad_part_7")
        assert not any("cad_part_7" in (pair.attrib.get("geom1"), pair.attrib.get("geom2"))
                       for pair in robot_internal_pairs(disabled_model))
        print("Collision-disabled fixed fork skips explicit floor and frame pairs")
        setup["partAssignments"][fixed_fork_path] = 6
        setup.pop("partOverrides", None)
        setup_path.write_text(json.dumps(setup), encoding="utf-8")

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
        assert len(floor_scrape_pairs(model)) == 3
        assert len(robot_internal_pairs(model)) == 1
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
        model = ET.fromstring(generated)
        assert len(floor_scrape_pairs(model)) == 3
        assert len(robot_internal_pairs(model)) == 0
        print("Per-fork main-frame collision toggle passed")

        hole_root = root / "hole_test"
        ring_file = "Parts/Weapon/ring.stl"
        frame_file = "Parts/Frame/frame.stl"
        # Weapon is the axis destination, but its mesh has no hole. The only
        # selectable round reference is Frame, which remains outside Weapon.
        tetra_stl(hole_root / ring_file)
        chamfered_ring_stl(hole_root / frame_file)
        hole_manifest = {
            "schemaVersion": 2,
            "designName": "Synthetic hole axis test",
            "unitToMeters": 0.001,
            "parts": [
                {"file": ring_file, "partName": "Ring", "bodyName": "Weapon/Ring",
                 "occurrencePath": ["Weapon", "Ring"], "massKg": 0.01,
                 "unitToMeters": 0.001, "positionFusionMeters": [0.10, 0.10, 0],
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
        assert "Outside-assembly reference part fitted Weapon axis without reassignment" in result.stdout
        result = subprocess.run([str(exe), "--test-axis-point", str(hole_manifest_path),
                                 "1", "1", "0.010", "0", "0.004"],
                                text=True, capture_output=True)
        print(result.stdout, end="")
        print(result.stderr, end="", file=sys.stderr)
        if result.returncode:
            raise SystemExit(result.returncode)
        assert "highlighted a 64-point round rim" in result.stdout

        result = subprocess.run([str(exe), "--validate-bundle", str(hole_manifest_path)],
                                text=True, capture_output=True)
        print(result.stdout, end="")
        print(result.stderr, end="", file=sys.stderr)
        if result.returncode:
            raise SystemExit(result.returncode)
        result = subprocess.run([str(exe), "--test-axis-hover",
                                 str(hole_root / "combatsim_preview.xml"),
                                 str(hole_manifest_path)], text=True, capture_output=True)
        print(result.stdout, end="")
        print(result.stderr, end="", file=sys.stderr)
        if result.returncode:
            raise SystemExit(result.returncode)
        assert "CAD-only preview rays" in result.stdout
        assert "reference part 1" in result.stdout
        assert "highlight loop" in result.stdout

        setup = json.loads(setup_path.read_text(encoding="utf-8"))
        wheel_layout = [
            ([-0.1, -0.04, 0], [-1, 0, 0]),
            ([-0.1, 0.04, 0], [1, 0, 0]),
            ([0.1, -0.04, 0], [-1, 0, 0]),
            ([0.1, 0.04, 0], [1, 0, 0]),
        ]
        for assembly, (pivot, axis) in zip(setup["assemblies"][1:5], wheel_layout):
            assembly["pivotMeters"] = pivot
            assembly["axis"] = axis
        setup_path.write_text(json.dumps(setup), encoding="utf-8")
        result = subprocess.run([str(exe), "--validate-bundle", str(root / "manifest.json")],
                                text=True, capture_output=True)
        print(result.stdout, end="")
        print(result.stderr, end="", file=sys.stderr)
        if result.returncode:
            raise SystemExit(result.returncode)
        assert "forward signs L=[1,-1] R=[1,-1]" in result.stdout
        oriented_model = ET.parse(root / "combatsim_generated.xml").getroot()
        player_quat = [float(value) for value in
                       oriented_model.find(".//body[@name='Player_Robot']").attrib["quat"].split()]
        assert abs(player_quat[0] - math.sqrt(0.5)) < 1e-6
        assert abs(player_quat[3] + math.sqrt(0.5)) < 1e-6
        setup_path = root / "combatsim_setup.json"
        setup = json.loads(setup_path.read_text(encoding="utf-8"))
        setup["botRotationDegrees"] = 30.0
        setup_path.write_text(json.dumps(setup), encoding="utf-8")
        result = subprocess.run([str(exe), "--validate-bundle", str(root / "manifest.json")],
                                text=True, capture_output=True)
        print(result.stdout, end="")
        print(result.stderr, end="", file=sys.stderr)
        if result.returncode:
            raise SystemExit(result.returncode)
        rotated_model = ET.parse(root / "combatsim_generated.xml").getroot()
        rotated_quat = [float(value) for value in
                        rotated_model.find(".//body[@name='Player_Robot']").attrib["quat"].split()]
        assert abs(rotated_quat[0] - math.sqrt(3) / 2) < 1e-6
        assert abs(rotated_quat[3] + 0.5) < 1e-6
        setup["botRotationDegrees"] = 0.0
        setup_path.write_text(json.dumps(setup), encoding="utf-8")
        result = subprocess.run([str(exe), "--validate-bundle", str(root / "manifest.json")],
                                text=True, capture_output=True)
        if result.returncode:
            print(result.stdout, end="")
            print(result.stderr, end="", file=sys.stderr)
            raise SystemExit(result.returncode)
        print("Configured whole-bot yaw rotates the root pose while preserving the fitted CAD axes")
        print("Wheel-layout heading, mirrored axle signs, and face-to-face spawn orientation passed")

        long_root = root / "long_path_case"
        long_components = ["occurrence_{:02d}_{}".format(i, "x" * 20)
                           for i in range(10)]
        long_file = Path("Parts", *long_components, "0000_frame.stl")
        tetra_stl(long_root / long_file)
        long_manifest = {
            "schemaVersion": 2,
            "designName": "Long mesh path test",
            "unitToMeters": 0.001,
            "parts": [{"file": long_file.as_posix(), "partName": "Frame",
                       "bodyName": "Frame", "occurrencePath": ["Frame"],
                       "massKg": 0.35, "unitToMeters": 0.001,
                       "positionFusionMeters": [0, 0, 0],
                       "rotationFusionWxyz": [1, 0, 0, 0]}],
        }
        long_manifest_path = long_root / "manifest.json"
        long_manifest_path.parent.mkdir(parents=True, exist_ok=True)
        long_manifest_path.write_text(json.dumps(long_manifest), encoding="utf-8")
        result = subprocess.run([str(exe), "--validate-bundle", str(long_manifest_path)],
                                text=True, capture_output=True)
        print(result.stdout, end="")
        print(result.stderr, end="", file=sys.stderr)
        if result.returncode:
            raise SystemExit(result.returncode)
        assert "CAD wheel drive mapping:" in result.stdout
        assert "CAD axis-fit tests skipped" in result.stdout
        long_cache = long_root / "Parts" / "_CombatSimCache"
        cached_meshes = list(long_cache.rglob("mesh_*.stl"))
        assert len(cached_meshes) == 1, "long Windows paths were not copied to the short asset cache"
        preview_mesh = ET.parse(long_root / "combatsim_preview.xml").find(".//mesh")
        assert preview_mesh is not None
        assert "_CombatSimCache" in preview_mesh.attrib["file"]
        assert "mesh_0.stl" in preview_mesh.attrib["file"]
        print("Windows long-path CAD mesh import and MuJoCo asset loading passed")

        disk_model_before = (long_root / "combatsim_generated.xml").read_bytes()
        disk_preview_before = (long_root / "combatsim_preview.xml").read_bytes()
        result = subprocess.run([str(exe), "--test-memory-bundle", str(long_manifest_path)],
                                text=True, capture_output=True)
        print(result.stdout, end="")
        print(result.stderr, end="", file=sys.stderr)
        if result.returncode:
            raise SystemExit(result.returncode)
        assert "In-memory CAD passed:" in result.stdout
        assert (long_root / "combatsim_generated.xml").read_bytes() == disk_model_before
        assert (long_root / "combatsim_preview.xml").read_bytes() == disk_preview_before
        print("Received CAD stayed in RAM and did not rewrite bundle-side model files")

        result = subprocess.run([str(exe), "--test-lan-session", str(root / "manifest.json"),
                                 str(opponent_manifest)],
                                text=True, capture_output=True, timeout=30)
        print(result.stdout, end="")
        print(result.stderr, end="", file=sys.stderr)
        if result.returncode:
            raise SystemExit(result.returncode)
        assert "LAN self-test passed:" in result.stdout


if __name__ == "__main__":
    main(Path(sys.argv[1]), Path(sys.argv[2]) if len(sys.argv) > 2 else None)
