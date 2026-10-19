"""Export a Fusion 360 design as separate MuJoCo-ready CAD parts.

Install from Fusion 360: Utilities > Add-ins > Scripts and Add-ins > + > select
the CombatSimExport folder. Run CombatSimExport while the robot design is open.
The output directory contains Parts/*.stl and a manifest.json sidecar.
"""

import json
import os
import re
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
        }
    translation = transform.translation
    return {
        "positionMeters": [translation.x * 0.01, translation.z * 0.01, translation.y * 0.01],
        "rotationUnity": _quat_from_transform(transform),
    }


def _body_metadata(body):
    material_name = "Unknown / Custom"
    density = 0.0
    mass = 0.0
    try:
        material = body.material
        if material:
            material_name = material.name
    except Exception:
        pass
    try:
        properties = body.physicalProperties
        mass = float(properties.mass)  # Fusion API mass is kilograms.
        density = float(properties.density) * 1000000.0  # kg/cm^3 -> kg/m^3.
    except Exception:
        pass
    return material_name, density, mass


def _export_body(export_manager, body, filepath, units):
    options = export_manager.createSTLExportOptions(body, filepath)
    if not options:
        raise RuntimeError("Fusion could not create STL export options for " + body.name)
    # Fusion initializes STL export units from the active design. The sidecar
    # records that same unit so Unity can convert the unitless STL coordinates.
    options.isBinaryFormat = True
    options.sendToPrintUtility = False
    if not export_manager.execute(options):
        raise RuntimeError("Fusion failed to export " + body.name)
    if not os.path.isfile(filepath) or os.path.getsize(filepath) <= 84:
        raise RuntimeError(
            "Fusion reported success but did not write a valid STL for " + body.name +
            ". Expected file: " + filepath
        )


def _append_part(parts, export_manager, body, body_name, transform, folder, index, units, unit_to_meters):
    if not body.isSolid:
        return
    part_name = body.name
    filename = "Parts/{:04d}_{}.stl".format(index, _safe_name(part_name))
    filepath = os.path.join(folder, filename.replace("/", os.sep))
    os.makedirs(os.path.dirname(filepath), exist_ok=True)
    _export_body(export_manager, body, filepath, units)
    material_name, density, mass = _body_metadata(body)
    transform_fields = _transform_data(transform)
    parts.append({
        "bodyName": body_name,
        "partName": part_name,
        "file": filename.replace(os.sep, "/"),
        "materialName": material_name,
        "densityKgPerM3": density,
        "massKg": mass,
        "frictionEstimate": 0.0,
        "unitToMeters": unit_to_meters,
        "positionMeters": transform_fields["positionMeters"],
        "rotationUnity": transform_fields["rotationUnity"],
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
            _append_part(parts, export_manager, body, root.name, None,
                         output_folder, index, units, unit_to_meters)
            index += 1

        occurrences = root.allOccurrences
        for occurrence_index in range(occurrences.count):
            occurrence = occurrences.item(occurrence_index)
            for body_index in range(occurrence.bRepBodies.count):
                proxy_body = occurrence.bRepBodies.item(body_index)
                native_body = proxy_body.nativeObject or proxy_body
                _append_part(parts, export_manager, native_body, occurrence.name,
                occurrence.transform2, output_folder, index, units,
                             unit_to_meters)
                index += 1

        manifest = {
            "schemaVersion": 1,
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
