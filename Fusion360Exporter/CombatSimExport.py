"""Export a Fusion 360 design as separate MuJoCo-ready CAD parts.

Install from Fusion 360: Utilities > Add-ins > Scripts and Add-ins > + > select
the CombatSimExport folder. Run CombatSimExport while the robot design is open.
The output directory contains Parts/*.stl and a manifest.json sidecar.
"""

import json
import os
import re
import struct
import traceback

import adsk.core
import adsk.fusion


def _safe_name(value):
    value = re.sub(r"[^A-Za-z0-9._-]+", "_", str(value)).strip("._")
    return value or "Part"


def _unit_to_meters(unit_name):
    unit = str(unit_name).lower().replace('"', "").strip()
    table = {
        "mm": 0.001, "millimeter": 0.001, "millimeters": 0.001,
        "cm": 0.01, "centimeter": 0.01, "centimeters": 0.01,
        "m": 1.0, "meter": 1.0, "meters": 1.0,
        "in": 0.0254, "inch": 0.0254, "inches": 0.0254,
        "ft": 0.3048, "foot": 0.3048, "feet": 0.3048,
    }
    for key, value in table.items():
        if unit == key or unit.startswith(key + " "):
            return value
    raise ValueError("Unsupported Fusion length unit: " + str(unit_name))


def _quat_from_transform(transform):
    # Fusion rotation matrices use a right-handed z-up basis. Convert the
    # quaternion into the Unity y-up basis used by the importer.
    m = [[transform.getCell(row, col) for col in range(3)] for row in range(3)]
    trace = m[0][0] + m[1][1] + m[2][2]
    if trace > 0.0:
        s = (trace + 1.0) ** 0.5 * 2.0
        qw = 0.25 * s
        qx = (m[2][1] - m[1][2]) / s
        qy = (m[0][2] - m[2][0]) / s
        qz = (m[1][0] - m[0][1]) / s
    elif m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = (1.0 + m[0][0] - m[1][1] - m[2][2]) ** 0.5 * 2.0
        qw = (m[2][1] - m[1][2]) / s
        qx = 0.25 * s
        qy = (m[0][1] + m[1][0]) / s
        qz = (m[0][2] + m[2][0]) / s
    elif m[1][1] > m[2][2]:
        s = (1.0 + m[1][1] - m[0][0] - m[2][2]) ** 0.5 * 2.0
        qw = (m[0][2] - m[2][0]) / s
        qx = (m[0][1] + m[1][0]) / s
        qy = 0.25 * s
        qz = (m[1][2] + m[2][1]) / s
    else:
        s = (1.0 + m[2][2] - m[0][0] - m[1][1]) ** 0.5 * 2.0
        qw = (m[1][0] - m[0][1]) / s
        qx = (m[0][2] + m[2][0]) / s
        qy = (m[1][2] + m[2][1]) / s
        qz = 0.25 * s
    # MjEngineTool's mapping: Unity quaternion = (x, z, y, -w).
    return [qx, qz, qy, -qw]


def _transform_data(transform):
    if transform is None:
        return {
            "positionMeters": [0.0, 0.0, 0.0],
            "rotationUnity": [0.0, 0.0, 0.0, 1.0],
            "positionFusionMeters": [0.0, 0.0, 0.0],
            "rotationFusionWxyz": [1.0, 0.0, 0.0, 0.0],
        }
    translation = transform.translation
    unity_quat = _quat_from_transform(transform)
    return {
        "positionMeters": [translation.x * 0.01, translation.z * 0.01, translation.y * 0.01],
        "rotationUnity": unity_quat,
        "positionFusionMeters": [translation.x * 0.01, translation.y * 0.01, translation.z * 0.01],
        "rotationFusionWxyz": [-unity_quat[3], unity_quat[0], unity_quat[2], unity_quat[1]],
    }


def _body_metadata(body):
    material_name = "Unknown / Custom"
    density = 0.0
    mass = 0.0
    volume = 0.0
    center = None
    try:
        material = body.material or body.parentComponent.material
        if material:
            material_name = material.name
    except Exception:
        pass
    try:
        properties = body.physicalProperties
        mass = float(properties.mass)  # Fusion API mass is kilograms.
        density = float(properties.density) * 1000000.0  # kg/cm^3 -> kg/m^3.
        volume = float(body.volume) * 0.000001  # cm^3 -> m^3.
        point = properties.centerOfMass
        center = [point.x * 0.01, point.y * 0.01, point.z * 0.01]
    except Exception:
        pass
    return material_name, density, mass, volume, center


def _appearance_metadata(body, occurrence=None):
    appearance = None
    try:
        if occurrence:
            appearance = occurrence.appearance
        if not appearance:
            appearance = body.appearance
        if not appearance:
            material = body.material or body.parentComponent.material
            if material:
                appearance = material.appearance
    except Exception:
        pass
    result = {"appearanceName": "", "colorRgba": None, "roughness": None}
    if not appearance:
        return result
    try:
        result["appearanceName"] = appearance.name
        color = appearance.color
        if color:
            result["colorRgba"] = [color.red / 255.0, color.green / 255.0,
                                   color.blue / 255.0, color.opacity / 255.0]
    except Exception:
        pass
    if result["colorRgba"] is None:
        for slot in ("opaque_albedo", "metal_f0", "transparent_color"):
            try:
                prop = appearance.appearanceProperties.itemById(slot)
                color = adsk.core.ColorProperty.cast(prop).value
                result["colorRgba"] = [color.red / 255.0, color.green / 255.0,
                                       color.blue / 255.0, color.opacity / 255.0]
                break
            except Exception:
                pass
    try:
        result["roughness"] = float(appearance.roughness)
    except Exception:
        try:
            prop = appearance.appearanceProperties.itemById("surface_roughness")
            result["roughness"] = float(adsk.core.FloatProperty.cast(prop).value)
        except Exception:
            pass
    return result


def _material_properties(body):
    exported = []
    try:
        material = body.material or body.parentComponent.material
        properties = material.materialProperties
    except Exception:
        return exported
    for index in range(properties.count):
        prop = properties.item(index)
        try:
            value = None
            units = ""
            float_prop = adsk.core.FloatProperty.cast(prop)
            if float_prop and not float_prop.hasMultipleValues:
                value = float(float_prop.value)
                units = float_prop.units
            else:
                int_prop = adsk.core.IntegerProperty.cast(prop)
                if int_prop:
                    value = int(int_prop.value)
                else:
                    string_prop = adsk.core.StringProperty.cast(prop)
                    if string_prop:
                        value = string_prop.value
                    else:
                        bool_prop = adsk.core.BooleanProperty.cast(prop)
                        if bool_prop:
                            value = bool(bool_prop.value)
            if value is not None:
                exported.append({"id": prop.id, "name": prop.name,
                                 "value": value, "units": units})
        except Exception:
            # Some Fusion library properties are compound or texture-backed.
            pass
    return exported


def _valid_binary_stl(filepath):
    if not os.path.isfile(filepath):
        return False
    size = os.path.getsize(filepath)
    if size < 134:
        return False
    with open(filepath, "rb") as stl:
        stl.seek(80)
        triangle_count = struct.unpack("<I", stl.read(4))[0]
    return triangle_count > 0 and size == 84 + 50 * triangle_count


def _write_mesh_stl(body, filepath, unit_to_meters):
    # Fusion's mesh coordinates are centimeters. Match the design-unit STL
    # coordinates expected by the bundle manifest and native importer.
    calculator = body.meshManager.createMeshCalculator()
    if not calculator:
        raise RuntimeError("Fusion could not create a mesh calculator for " + body.name)
    calculator.setQuality(adsk.fusion.TriangleMeshQualityOptions.HighQualityTriangleMesh)
    mesh = calculator.calculate()
    if not mesh or mesh.triangleCount < 1:
        raise RuntimeError("Fusion could not tessellate " + body.name)
    points = mesh.nodeCoordinates
    indices = mesh.nodeIndices
    if len(indices) != mesh.triangleCount * 3:
        raise RuntimeError("Fusion returned invalid mesh indices for " + body.name)
    scale = 0.01 / unit_to_meters
    with open(filepath, "wb") as stl:
        stl.write(b"CombatSim Fusion mesh".ljust(80, b"\0"))
        stl.write(struct.pack("<I", mesh.triangleCount))
        for i in range(0, len(indices), 3):
            vertices = []
            for node_index in indices[i:i + 3]:
                p = points[node_index]
                vertices.append((p.x * scale, p.y * scale, p.z * scale))
            a, b, c = vertices
            ab = (b[0] - a[0], b[1] - a[1], b[2] - a[2])
            ac = (c[0] - a[0], c[1] - a[1], c[2] - a[2])
            normal = (ab[1] * ac[2] - ab[2] * ac[1],
                      ab[2] * ac[0] - ab[0] * ac[2],
                      ab[0] * ac[1] - ab[1] * ac[0])
            length = sum(v * v for v in normal) ** 0.5
            if length:
                normal = tuple(v / length for v in normal)
            stl.write(struct.pack("<12fH", *(normal + a + b + c), 0))


def _export_body(export_manager, body, filepath, units, unit_to_meters):
    # A previous run may have left an STL behind. Never mistake it for this run.
    if os.path.exists(filepath):
        os.remove(filepath)
    exported = False
    try:
        options = export_manager.createSTLExportOptions(body, filepath)
        if options:
            # Fusion initializes STL export units from the active design.
            options.isBinaryFormat = True
            options.sendToPrintUtility = False
            exported = export_manager.execute(options)
    except Exception:
        # The B-Rep mesh path below can still export this body.
        pass
    if not exported or not _valid_binary_stl(filepath):
        # Fusion can report success but omit hidden bodies from its STL export.
        # Tessellating the B-Rep works independently of browser visibility.
        _write_mesh_stl(body, filepath, unit_to_meters)
    if not _valid_binary_stl(filepath):
        raise RuntimeError(
            "Fusion could not write a valid STL for " + body.name +
            ". Expected file: " + filepath
        )


def _append_part(parts, export_manager, body, body_name, occurrence_path,
                 transform, folder, index, units, unit_to_meters, occurrence=None):
    if not body.isSolid:
        return
    part_name = body.name
    path_folder = "/".join(_safe_name(name) for name in occurrence_path) or "Root"
    filename = "Parts/{}/{:04d}_{}.stl".format(path_folder, index, _safe_name(part_name))
    filepath = os.path.join(folder, filename.replace("/", os.sep))
    os.makedirs(os.path.dirname(filepath), exist_ok=True)
    _export_body(export_manager, body, filepath, units, unit_to_meters)
    material_name, density, mass, volume, center = _body_metadata(body)
    appearance = _appearance_metadata(body, occurrence)
    material_properties = _material_properties(body)
    transform_fields = _transform_data(transform)
    parts.append({
        "bodyName": body_name,
        "occurrencePath": occurrence_path,
        "partName": part_name,
        "file": filename.replace(os.sep, "/"),
        "materialName": material_name,
        "densityKgPerM3": density,
        "massKg": mass,
        "volumeM3": volume,
        "centerOfMassLocalMeters": center,
        "appearanceName": appearance["appearanceName"],
        "colorRgba": appearance["colorRgba"],
        "roughness": appearance["roughness"],
        "materialProperties": material_properties,
        "frictionEstimate": 0.0,
        "unitToMeters": unit_to_meters,
        "positionMeters": transform_fields["positionMeters"],
        "rotationUnity": transform_fields["rotationUnity"],
        "positionFusionMeters": transform_fields["positionFusionMeters"],
        "rotationFusionWxyz": transform_fields["rotationFusionWxyz"],
    })


def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        design = adsk.fusion.Design.cast(app.activeProduct)
        if not design:
            ui.messageBox("Open a Fusion 360 design before running this exporter.")
            return

        folder_dialog = ui.createFolderDialog()
        folder_dialog.title = "Choose CombatSim bundle output folder"
        if folder_dialog.showDialog() != adsk.core.DialogResults.DialogOK:
            return
        root_folder = folder_dialog.folder
        bundle_name = _safe_name(design.rootComponent.name)
        output_folder = os.path.join(root_folder, bundle_name + "_MuJoCo")
        os.makedirs(output_folder, exist_ok=True)

        units = design.unitsManager.defaultLengthUnits
        unit_to_meters = _unit_to_meters(units)
        export_manager = design.exportManager
        parts = []
        index = 0

        root = design.rootComponent
        for i in range(root.bRepBodies.count):
            body = root.bRepBodies.item(i)
            _append_part(parts, export_manager, body, root.name, [], None,
                         output_folder, index, units, unit_to_meters)
            index += 1

        occurrences = root.allOccurrences
        for occurrence_index in range(occurrences.count):
            occurrence = occurrences.item(occurrence_index)
            try:
                occurrence_path = occurrence.fullPathName.split("+")
            except Exception:
                occurrence_path = [occurrence.name]
            for body_index in range(occurrence.bRepBodies.count):
                proxy_body = occurrence.bRepBodies.item(body_index)
                native_body = proxy_body.nativeObject or proxy_body
                _append_part(parts, export_manager, native_body, occurrence.name,
                occurrence_path, occurrence.transform2, output_folder, index, units,
                             unit_to_meters, occurrence)
                index += 1

        manifest = {
            "schemaVersion": 2,
            "designName": design.rootComponent.name,
            "lengthUnit": units,
            "unitToMeters": unit_to_meters,
            "weaponMaxRpm": 30000.0,
            "weaponSpinUpSeconds": 1.5,
            "parts": parts,
        }
        manifest_path = os.path.join(output_folder, "manifest.json")
        with open(manifest_path, "w", encoding="utf-8") as manifest_file:
            json.dump(manifest, manifest_file, indent=2, ensure_ascii=False)

        ui.messageBox(
            "CombatSim Fusion export finished.\n\n" + output_folder +
            "\n\nParts exported: " + str(len(parts)) +
            "\nImport manifest.json from the CombatSim Hub."
        )
    except:
        if ui:
            ui.messageBox("CombatSim export failed:\n" + traceback.format_exc())
