"""Export a Fusion 360 design as separate MuJoCo-ready CAD parts.

Install from Fusion 360: Utilities > Add-ins > Scripts and Add-ins > + > select
the CombatSimExport folder. Run CombatSimExport while the robot design is open.
The output directory contains selectable Parts/*.stl meshes, smooth render
Parts/*.obj meshes with UVs/normals, supported appearance maps in Textures/,
and a manifest.json sidecar.
"""

import json
import math
import os
import re
import shutil
import struct
import traceback

import adsk.core
import adsk.fusion


def _safe_name(value):
    value = re.sub(r"[^A-Za-z0-9._-]+", "_", str(value)).strip("._")
    return value or "Part"


def _mesh_filename(index, part_name):
    # Fusion occurrence paths can be hundreds of characters deep. Keep the
    # physical mesh path short; occurrencePath in the manifest still carries
    # the complete assembly tree for the editor.
    short_name = _safe_name(part_name)[:64].rstrip("._") or "Part"
    return "Parts/{:04d}_{}.stl".format(index, short_name)


def _render_mesh_filename(index, part_name):
    return _mesh_filename(index, part_name)[:-4] + ".obj"


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


def _fusion_quat_wxyz_from_transform(transform):
    # Fusion uses a right-handed z-up basis. Keep its native axis order and
    # serialize the quaternion as WXYZ for MuJoCo.
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
    return [qw, qx, qy, qz]


def _transform_data(transform):
    if transform is None:
        return {
            "positionFusionMeters": [0.0, 0.0, 0.0],
            "rotationFusionWxyz": [1.0, 0.0, 0.0, 0.0],
        }
    translation = transform.translation
    return {
        "positionFusionMeters": [translation.x * 0.01, translation.y * 0.01, translation.z * 0.01],
        "rotationFusionWxyz": _fusion_quat_wxyz_from_transform(transform),
    }


def _triangle_mesh(body):
    """Get a triangle mesh from either a parametric body or an imported mesh body."""
    try:
        calculator = body.meshManager.createMeshCalculator()
    except Exception:
        calculator = None
    if calculator:
        calculator.setQuality(adsk.fusion.TriangleMeshQualityOptions.HighQualityTriangleMesh)
        mesh = calculator.calculate()
    else:
        # Fusion stores imported STL/OBJ geometry as MeshBody objects. They do
        # not have a BRep meshManager, but displayMesh is already triangulated
        # and includes the node normals and texture coordinates.
        mesh = body.displayMesh
    if not mesh or mesh.triangleCount < 1:
        raise RuntimeError("Fusion could not tessellate " + body.name)
    return mesh


def _yield_to_fusion():
    """Let Fusion repaint and service its UI during long Python-side exports."""
    try:
        adsk.doEvents()
    except Exception:
        # Unit tests and older Fusion builds may not expose the event pump.
        pass


def _mesh_volume_and_center(mesh):
    """Estimate the volume and centroid of a closed, oriented triangle mesh.

    Fusion's mesh-body API does not expose PhysicalProperties. The signed
    tetrahedron sum gives accurate mass properties for watertight oriented
    meshes; open meshes are left for a user mass override in the editor.
    Coordinates from Fusion's PolygonMesh are in centimeters.
    """
    points = mesh.nodeCoordinates
    indices = mesh.nodeIndices
    if len(indices) != mesh.triangleCount * 3:
        return 0.0, None
    signed_volume_cm3 = 0.0
    centroid_sum = [0.0, 0.0, 0.0]
    for triangle_number, offset in enumerate(range(0, len(indices), 3)):
        ia, ib, ic = indices[offset:offset + 3]
        if min(ia, ib, ic) < 0 or max(ia, ib, ic) >= len(points):
            return 0.0, None
        a, b, c = points[ia], points[ib], points[ic]
        cross = (b.y * c.z - b.z * c.y,
                 b.z * c.x - b.x * c.z,
                 b.x * c.y - b.y * c.x)
        volume = (a.x * cross[0] + a.y * cross[1] + a.z * cross[2]) / 6.0
        signed_volume_cm3 += volume
        for axis, value in enumerate((a.x + b.x + c.x,
                                      a.y + b.y + c.y,
                                      a.z + b.z + c.z)):
            centroid_sum[axis] += volume * value / 4.0
        if triangle_number % 4096 == 4095:
            _yield_to_fusion()
    if abs(signed_volume_cm3) < 1e-12:
        return 0.0, None
    center = [value / signed_volume_cm3 * 0.01 for value in centroid_sum]
    return abs(signed_volume_cm3) * 1e-6, center


def _mesh_body_density(body):
    """Read physical density from the mesh body's assigned Fusion material."""
    try:
        material = body.material or body.parentComponent.material
        props = material.materialProperties
    except Exception:
        return 0.0
    for index in range(props.count):
        try:
            prop = props.item(index)
            identifier = str(prop.id).lower()
            name = str(prop.name).lower()
            if "density" not in identifier and "density" not in name:
                continue
            density_prop = adsk.core.FloatProperty.cast(prop)
            if not density_prop or density_prop.hasMultipleValues:
                continue
            value = float(density_prop.value)
            units = str(density_prop.units).lower()
            if "kilogrampercubicmeter" in units or "kg/m" in units or "kg/m3" in units:
                return value
            if "kilogrampercubiccentimeter" in units or "kg/cm" in units or value < 1.0:
                return value * 1000000.0
            return value
        except Exception:
            continue
    return 0.0


def _body_metadata(body, mesh_body=False, mesh=None):
    material_name = "Unknown / Custom"
    density = 0.0
    mass = 0.0
    volume = 0.0
    center = None
    inertia_tensor = None
    properties = None
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
    if mesh_body and not properties:
        try:
            if mesh is None:
                mesh = _triangle_mesh(body)
            try:
                closed_and_oriented = bool(body.isClosed and body.isOriented)
            except Exception:
                closed_and_oriented = False
            if closed_and_oriented:
                volume, center = _mesh_volume_and_center(mesh)
                density = _mesh_body_density(body)
                mass = volume * density
        except Exception:
            pass
    if properties:
        try:
            moments_ok, i1, i2, i3 = properties.getPrincipalMomentsOfInertia()
            axes_ok, x_axis, y_axis, z_axis = properties.getPrincipalAxes()
            moments = (float(i1), float(i2), float(i3))
            axes = (
                (float(x_axis.x), float(x_axis.y), float(x_axis.z)),
                (float(y_axis.x), float(y_axis.y), float(y_axis.z)),
                (float(z_axis.x), float(z_axis.y), float(z_axis.z)),
            )
            if moments_ok and axes_ok and all(value >= 0.0 for value in moments):
                # Principal moments are about the body's center of mass.
                # Build the centroidal tensor in native body coordinates;
                # Fusion reports kg*cm^2, while the bundle uses kg*m^2.
                matrix = [[sum(moments[k] * axes[k][row] * axes[k][col]
                               for k in range(3)) * 0.0001
                           for col in range(3)] for row in range(3)]
                inertia_tensor = [matrix[0][0], matrix[1][1], matrix[2][2],
                                  matrix[0][1], matrix[0][2], matrix[1][2]]
                if not all(math.isfinite(value) for value in inertia_tensor):
                    inertia_tensor = None
        except Exception:
            # Older/unsupported Fusion builds still export mass and COM;
            # the importer can use its bounding-box inertia estimate.
            pass
    return material_name, density, mass, volume, center, inertia_tensor


def _resolve_appearance(body, occurrence=None):
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
    return appearance


def _appearance_is_transparent(appearance):
    try:
        properties = appearance.appearanceProperties
        for property_id in ("transparent_color", "transparent_ior"):
            if properties.itemById(property_id):
                return True
    except Exception:
        pass
    name = str(getattr(appearance, "name", "")).lower()
    return any(token in name for token in ("transparent", "glass", "clear"))


def _appearance_rgba(color, appearance, body, occurrence):
    alpha = float(color.opacity) / 255.0
    if alpha <= 0.0 and not _appearance_is_transparent(appearance):
        visible_opacities = []
        for target in (body, occurrence):
            if target is None:
                continue
            try:
                opacity = float(target.visibleOpacity)
                if math.isfinite(opacity) and 0.0 <= opacity <= 1.0:
                    visible_opacities.append(opacity)
            except Exception:
                pass
        # Fusion appearances can return zero in the RGBA color slot even for
        # opaque plastics/rubber. Use the body's displayed opacity when present;
        # otherwise treat a non-transparent material as opaque.
        alpha = min(visible_opacities) if visible_opacities else 1.0
    return [color.red / 255.0, color.green / 255.0,
            color.blue / 255.0, alpha]


def _appearance_metadata(body, occurrence=None, appearance=None):
    if appearance is None:
        appearance = _resolve_appearance(body, occurrence)
    result = {"appearanceName": "", "colorRgba": None, "roughness": None}
    if not appearance:
        return result
    try:
        result["appearanceName"] = appearance.name
        color = appearance.color
        if color:
            result["colorRgba"] = _appearance_rgba(
                color, appearance, body, occurrence)
    except Exception:
        pass
    if result["colorRgba"] is None:
        for slot in ("opaque_albedo", "metal_f0", "transparent_color"):
            try:
                prop = appearance.appearanceProperties.itemById(slot)
                color = adsk.core.ColorProperty.cast(prop).value
                result["colorRgba"] = _appearance_rgba(
                    color, appearance, body, occurrence)
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


def _copy_appearance_textures(appearance, folder, part_index):
    """Copy Fusion's local copies of appearance maps into the portable bundle."""
    copied = {}
    if not appearance:
        return copied
    slots = (("rgb", "colorTexture"),
             ("normal", "normalTexture"),
             ("roughness", "roughnessTexture"))
    for role, property_name in slots:
        try:
            source = str(getattr(appearance, property_name) or "")
        except Exception:
            continue
        if not source or not os.path.isfile(source):
            continue
        extension = os.path.splitext(source)[1].lower()
        if extension not in (".png", ".jpg", ".jpeg"):
            continue
        relative = "Textures/{:04d}_{}{}".format(part_index, role, extension)
        destination = os.path.join(folder, relative.replace("/", os.sep))
        os.makedirs(os.path.dirname(destination), exist_ok=True)
        shutil.copyfile(source, destination)
        copied[role] = relative
    return copied


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


def _write_mesh_stl(body, filepath, unit_to_meters, mesh=None):
    # Fusion's mesh coordinates are centimeters. Match the design-unit STL
    # coordinates expected by the bundle manifest and native importer.
    if mesh is None:
        mesh = _triangle_mesh(body)
    points = mesh.nodeCoordinates
    indices = mesh.nodeIndices
    if len(indices) != mesh.triangleCount * 3:
        raise RuntimeError("Fusion returned invalid mesh indices for " + body.name)
    scale = 0.01 / unit_to_meters
    with open(filepath, "wb") as stl:
        stl.write(b"CombatSim Fusion mesh".ljust(80, b"\0"))
        stl.write(struct.pack("<I", mesh.triangleCount))
        batch_size = 4096
        batch = bytearray(batch_size * 50)
        batch_count = 0
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
            struct.pack_into("<12fH", batch, batch_count * 50,
                             *(normal + a + b + c), 0)
            batch_count += 1
            if batch_count == batch_size:
                stl.write(batch)
                batch_count = 0
                _yield_to_fusion()
        if batch_count:
            stl.write(memoryview(batch)[:batch_count * 50])


def _write_render_mesh_obj(body, filepath, unit_to_meters, mesh=None,
                           include_uv=True):
    """Write an OBJ with Fusion's per-node normals and a stable planar UV map.

    Binary STL stores only a normal per triangle. That makes every tessellation
    seam visible under directional lighting. OBJ preserves Fusion's smoothly
    varying surface normals and can carry texture coordinates for MuJoCo.
    """
    if mesh is None:
        mesh = _triangle_mesh(body)
    points = mesh.nodeCoordinates
    indices = mesh.nodeIndices
    normals = mesh.normalVectors
    try:
        texture_coordinates = mesh.textureCoordinates
    except Exception:
        texture_coordinates = []
    if len(indices) != mesh.triangleCount * 3:
        raise RuntimeError("Fusion returned invalid render mesh indices for " + body.name)
    if not points or any(index < 0 or index >= len(points) for index in indices):
        raise RuntimeError("Fusion returned invalid render mesh vertices for " + body.name)

    scale = 0.01 / unit_to_meters
    scaled = [(point.x * scale, point.y * scale, point.z * scale) for point in points]
    if len(normals) == len(points):
        normal_values = [(normal.x, normal.y, normal.z) for normal in normals]
    else:
        # Older Fusion builds can omit node normals. Accumulate area-weighted
        # face normals at shared nodes so the fallback still avoids STL-like
        # lighting changes at each triangle boundary.
        normal_values = [[0.0, 0.0, 0.0] for _ in points]
        for triangle_number, offset in enumerate(range(0, len(indices), 3)):
            a, b, c = (scaled[indices[offset + corner]] for corner in range(3))
            ab = (b[0] - a[0], b[1] - a[1], b[2] - a[2])
            ac = (c[0] - a[0], c[1] - a[1], c[2] - a[2])
            face_normal = (ab[1] * ac[2] - ab[2] * ac[1],
                           ab[2] * ac[0] - ab[0] * ac[2],
                           ab[0] * ac[1] - ab[1] * ac[0])
            for corner in range(3):
                target = normal_values[indices[offset + corner]]
                for axis in range(3):
                    target[axis] += face_normal[axis]
            if triangle_number % 4096 == 4095:
                _yield_to_fusion()
        normal_values = [tuple(value) for value in normal_values]

    normalized_normals = []
    for normal in normal_values:
        length = sum(component * component for component in normal) ** 0.5
        normalized_normals.append(tuple(component / length for component in normal)
                                  if length > 1e-12 else (0.0, 0.0, 1.0))
    normal_values = normalized_normals
    native_uvs = include_uv and len(texture_coordinates) == len(points)
    if include_uv and not native_uvs:
        bounds_min = [min(point[axis] for point in scaled) for axis in range(3)]
        bounds_max = [max(point[axis] for point in scaled) for axis in range(3)]
        bounds_span = [max(bounds_max[axis] - bounds_min[axis], 1e-12)
                       for axis in range(3)]
    else:
        bounds_min, bounds_span = (0.0, 0.0, 0.0), (1.0, 1.0, 1.0)
    with open(filepath, "w", encoding="ascii", newline="\n") as obj:
        buffer = []

        def flush_buffer():
            if buffer:
                obj.write("".join(buffer))
                buffer.clear()
                _yield_to_fusion()

        def add_line(line):
            buffer.append(line)
            if len(buffer) >= 12000:
                flush_buffer()

        add_line("# CombatSim Fusion render mesh; Fusion node normals preserved\n")
        for point in scaled:
            add_line("v {:.9g} {:.9g} {:.9g}\n".format(*point))
        for normal in normal_values:
            add_line("vn {:.9g} {:.9g} {:.9g}\n".format(*normal))
        if native_uvs:
            for uv in texture_coordinates:
                add_line("vt {:.9g} {:.9g}\n".format(uv.x, uv.y))
        elif not include_uv:
            # MuJoCo's OBJ loader accepts texture-coordinate indices even when
            # no maps are assigned. Keep one neutral UV per vertex so every
            # face shares indices without tripling the texture-coordinate data.
            for _point in points:
                add_line("vt 0 0\n")

        for offset in range(0, len(indices), 3):
            triangle_indices = indices[offset:offset + 3]
            if native_uvs or not include_uv:
                # Fusion's per-node UVs map directly to the shared mesh nodes.
                # Without a texture map, neutral coordinates use the same
                # compact one-coordinate-per-node representation.
                face_refs = ["{}/{}/{}".format(vertex_index + 1,
                                                vertex_index + 1,
                                                vertex_index + 1)
                             for vertex_index in triangle_indices]
            else:
                # If a textured body lacks native UVs, preserve the per-face
                # planar fallback so each triangle can choose its projection.
                a, b, c = (scaled[index] for index in triangle_indices)
                ab = (b[0] - a[0], b[1] - a[1], b[2] - a[2])
                ac = (c[0] - a[0], c[1] - a[1], c[2] - a[2])
                face_normal = (ab[1] * ac[2] - ab[2] * ac[1],
                               ab[2] * ac[0] - ab[0] * ac[2],
                               ab[0] * ac[1] - ab[1] * ac[0])
                dominant = max(range(3), key=lambda axis: abs(face_normal[axis]))
                u_axis, v_axis = ((1, 2), (0, 2), (0, 1))[dominant]
                face_refs = []
                for vertex_index in triangle_indices:
                    point = scaled[vertex_index]
                    u = (point[u_axis] - bounds_min[u_axis]) / bounds_span[u_axis]
                    v = (point[v_axis] - bounds_min[v_axis]) / bounds_span[v_axis]
                    uv_index = len(points) + (offset // 3) * 3 + len(face_refs) + 1
                    add_line("vt {:.9g} {:.9g}\n".format(u, v))
                    face_refs.append("{}/{}/{}".format(vertex_index + 1, uv_index,
                                                        vertex_index + 1))
            add_line("f {}\n".format(" ".join(face_refs)))
        flush_buffer()


def _export_body(export_manager, body, filepath, units, unit_to_meters,
                 mesh_body=False, mesh=None):
    # A previous run may have left an STL behind. Never mistake it for this run.
    if os.path.exists(filepath):
        os.remove(filepath)
    # Use one Fusion tessellation for both physical STL and smooth render OBJ.
    # A separate ExportManager STL call tessellates the same body a second time.
    if mesh is None:
        mesh = _triangle_mesh(body)
    _write_mesh_stl(body, filepath, unit_to_meters, mesh)
    if not _valid_binary_stl(filepath):
        raise RuntimeError(
            "Fusion could not write a valid STL for " + body.name +
            ". Expected file: " + filepath
        )
    return mesh


def _append_part(parts, export_manager, body, body_name, occurrence_path,
                 transform, folder, index, units, unit_to_meters, occurrence=None,
                 mesh_body=False, appearance_body=None):
    solid_body = False
    if not mesh_body:
        solid_body = _is_solid_body(body)
        if not solid_body:
            raise RuntimeError(
                "Non-solid B-Rep surface bodies are excluded from CombatSim export: " +
                body.name)
    part_name = body.name
    filename = _mesh_filename(index, part_name)
    filepath = os.path.join(folder, filename.replace("/", os.sep))
    os.makedirs(os.path.dirname(filepath), exist_ok=True)
    mesh = _triangle_mesh(body)
    material_name, density, mass, volume, center, inertia_tensor = _body_metadata(
        body, mesh_body, mesh)
    appearance_body = appearance_body or body
    fusion_appearance = _resolve_appearance(appearance_body, occurrence)
    appearance = _appearance_metadata(appearance_body, occurrence, fusion_appearance)
    textures = _copy_appearance_textures(fusion_appearance, folder, index)
    mesh = _export_body(export_manager, body, filepath, units, unit_to_meters,
                        mesh_body, mesh)
    render_filename = _render_mesh_filename(index, part_name)
    render_filepath = os.path.join(folder, render_filename.replace("/", os.sep))
    _write_render_mesh_obj(body, render_filepath, unit_to_meters, mesh,
                           include_uv=bool(textures))
    material_properties = _material_properties(body)
    transform_fields = _transform_data(transform)
    parts.append({
        "bodyName": body_name,
        "occurrencePath": occurrence_path,
        "partName": part_name,
        "geometryType": "meshBody" if mesh_body else
                        "brepBody" if solid_body else "surfaceBody",
        "file": filename.replace(os.sep, "/"),
        "renderFile": render_filename.replace(os.sep, "/"),
        "textures": textures,
        "materialName": material_name,
        "densityKgPerM3": density,
        "massKg": mass,
        "volumeM3": volume,
        "centerOfMassLocalMeters": center,
        "inertiaTensorLocalKgM2": inertia_tensor,
        "appearanceName": appearance["appearanceName"],
        "colorRgba": appearance["colorRgba"],
        "roughness": appearance["roughness"],
        "materialProperties": material_properties,
        "frictionEstimate": 0.0,
        "unitToMeters": unit_to_meters,
        "positionFusionMeters": transform_fields["positionFusionMeters"],
        "rotationFusionWxyz": transform_fields["rotationFusionWxyz"],
    })


def _occurrence_path(occurrence):
    try:
        full_path = str(occurrence.fullPathName)
        if full_path:
            return [item for item in full_path.split("+") if item]
    except Exception:
        pass
    try:
        return [str(occurrence.name)]
    except Exception:
        return ["<unavailable occurrence>"]


def _record_occurrence_skip(skipped, path, stage, error):
    if skipped is None:
        return
    description = str(error).replace("\r", " ").replace("\n", " ")
    path_prefix = "{}: ".format(" / ".join(path))
    if any(item.startswith(path_prefix) and item.endswith("({})".format(description))
           for item in skipped):
        return
    skipped.append("{}: {} ({})".format(" / ".join(path), stage, description))


def _progress_start(progress, ui, completed, total, label):
    if not progress:
        return
    try:
        ui.statusMessage = "CombatSim export: " + str(label)
        progress.progressValue = min(completed, max(1, total))
        _yield_to_fusion()
    except Exception:
        pass


def _progress_finish(progress, ui, completed, total):
    completed += 1
    if progress:
        try:
            progress.progressValue = min(completed, max(1, total))
        except Exception:
            pass
    _yield_to_fusion()
    return completed


def _is_solid_body(body):
    try:
        return bool(body.isSolid)
    except Exception:
        # Preserve bodies whose type Fusion cannot report; export errors are
        # handled by the normal per-body path.
        return True


def _body_type_counts(bodies):
    solid_count = 0
    surface_count = 0
    for index in range(bodies.count):
        try:
            body = bodies.item(index)
            if _is_solid_body(body):
                solid_count += 1
            else:
                surface_count += 1
        except Exception:
            # The body may become unavailable between counting and export.
            solid_count += 1
        if index % 256 == 255:
            _yield_to_fusion()
    return solid_count, surface_count


def _iter_occurrences(occurrences, skipped=None, parent_path=None):
    """Walk the browser occurrence tree without using Component.allOccurrences.

    Fusion's allOccurrences getter can raise an InternalValidationError on some
    designs even though the root and its occurrence collections are readable.
    Autodesk documents recursive traversal through childOccurrences as the
    assembly-tree traversal pattern.
    """
    parent_path = parent_path or []
    try:
        count = occurrences.count
    except Exception as error:
        _record_occurrence_skip(skipped, parent_path or ["<occurrences>"],
                                "could not read occurrence list", error)
        return
    for index in range(count):
        try:
            occurrence = occurrences.item(index)
        except Exception as error:
            _record_occurrence_skip(skipped, parent_path or ["<occurrences>"],
                                    "could not read occurrence {}".format(index), error)
            continue
        path = _occurrence_path(occurrence)
        yield occurrence
        try:
            children = occurrence.childOccurrences
            child_count = children.count if children else 0
        except Exception as error:
            _record_occurrence_skip(skipped, path,
                                    "could not inspect nested components", error)
            continue
        if child_count:
            yield from _iter_occurrences(children, skipped, path)


def run(context):
    ui = None
    progress_dialog = None
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
        skipped_items = []
        index = 0

        root = design.rootComponent
        ui.statusMessage = "CombatSim export: scanning assembly..."
        _yield_to_fusion()
        root_brep_bodies = root.bRepBodies
        root_brep_count = root_brep_bodies.count
        root_solid_count, root_surface_count = _body_type_counts(root_brep_bodies)
        root_mesh_bodies = None
        try:
            root_mesh_bodies = root.meshBodies
            root_mesh_count = root_mesh_bodies.count
        except Exception as error:
            root_mesh_count = 0
            _record_occurrence_skip(skipped_items, [root.name],
                                    "could not inspect mesh bodies", error)

        occurrences = list(_iter_occurrences(root.occurrences, skipped_items))
        total_work = root_solid_count + root_mesh_count
        total_surface_count = root_surface_count
        for occurrence in occurrences:
            occurrence_path = _occurrence_path(occurrence)
            try:
                solid_count, surface_count = _body_type_counts(occurrence.bRepBodies)
                total_work += solid_count
                total_surface_count += surface_count
            except Exception as error:
                _record_occurrence_skip(skipped_items, occurrence_path,
                                        "component is unavailable", error)
            try:
                total_work += occurrence.component.meshBodies.count
            except Exception as error:
                _record_occurrence_skip(skipped_items, occurrence_path,
                                        "could not inspect mesh bodies", error)
            _yield_to_fusion()

        skipped_surface_count = 0

        try:
            progress_dialog = ui.createProgressDialog()
            progress_dialog.isCancelButtonShown = False
            progress_dialog.show("CombatSim Fusion Export",
                                 "Exporting part %v of %m", 0,
                                 max(1, total_work), 0)
        except Exception:
            progress_dialog = None
        progress_count = 0

        for i in range(root_brep_count):
            body = root_brep_bodies.item(i)
            if not _is_solid_body(body):
                skipped_surface_count += 1
                continue
            _progress_start(progress_dialog, ui, progress_count, total_work, body.name)
            try:
                _append_part(parts, export_manager, body, root.name, [], None,
                             output_folder, index, units, unit_to_meters)
                index += 1
            finally:
                progress_count = _progress_finish(
                    progress_dialog, ui, progress_count, total_work)

        for body_index in range(root_mesh_count):
            _progress_start(progress_dialog, ui, progress_count, total_work,
                            "{} mesh body {}".format(root.name, body_index + 1))
            try:
                body = root_mesh_bodies.item(body_index)
                native_body = body.nativeObject or body
                _append_part(parts, export_manager, native_body, root.name, [], None,
                             output_folder, index, units, unit_to_meters,
                             mesh_body=True)
                index += 1
            except Exception as error:
                _record_occurrence_skip(
                    skipped_items, [root.name],
                    "could not export mesh body {}".format(body_index), error)
                continue
            finally:
                progress_count = _progress_finish(
                    progress_dialog, ui, progress_count, total_work)

        for occurrence in occurrences:
            occurrence_path = _occurrence_path(occurrence)
            try:
                occurrence_name = str(occurrence.name)
                bodies = occurrence.bRepBodies
                body_count = bodies.count
                transform = occurrence.transform2
            except Exception as error:
                _record_occurrence_skip(skipped_items, occurrence_path,
                                        "component is unavailable", error)
                continue
            for body_index in range(body_count):
                try:
                    proxy_body = bodies.item(body_index)
                    native_body = proxy_body.nativeObject or proxy_body
                except Exception as error:
                    _record_occurrence_skip(
                        skipped_items, occurrence_path,
                        "could not access body {}".format(body_index), error)
                    continue
                if not _is_solid_body(native_body):
                    skipped_surface_count += 1
                    continue
                _progress_start(progress_dialog, ui, progress_count, total_work,
                                "{} body {}".format(occurrence_name, body_index + 1))
                try:
                    _append_part(parts, export_manager, native_body, occurrence_name,
                                 occurrence_path, transform, output_folder, index,
                                 units, unit_to_meters, occurrence,
                                 appearance_body=proxy_body)
                    index += 1
                except Exception as error:
                    message = str(error).lower()
                    if ("referenced component is unavailable" in message or
                            ("external reference" in message and
                             ("broken" in message or "missing" in message))):
                        _record_occurrence_skip(
                            skipped_items, occurrence_path,
                            "component became unavailable during export", error)
                        continue
                    raise
                finally:
                    progress_count = _progress_finish(
                        progress_dialog, ui, progress_count, total_work)

            try:
                # MeshBody collections belong to Components. occurrence.component
                # is the assembly-context component for nested occurrences.
                mesh_bodies = occurrence.component.meshBodies
                mesh_count = mesh_bodies.count
            except Exception as error:
                _record_occurrence_skip(skipped_items, occurrence_path,
                                        "could not inspect mesh bodies", error)
                continue
            for body_index in range(mesh_count):
                _progress_start(progress_dialog, ui, progress_count, total_work,
                                "{} mesh body {}".format(occurrence_name, body_index + 1))
                try:
                    proxy_body = mesh_bodies.item(body_index)
                    native_body = proxy_body.nativeObject or proxy_body
                    _append_part(parts, export_manager, native_body, occurrence_name,
                                 occurrence_path, transform, output_folder, index,
                                 units, unit_to_meters, occurrence, mesh_body=True,
                                 appearance_body=proxy_body)
                    index += 1
                except Exception as error:
                    _record_occurrence_skip(
                        skipped_items, occurrence_path,
                        "could not export mesh body {}".format(body_index), error)
                    continue
                finally:
                    progress_count = _progress_finish(
                        progress_dialog, ui, progress_count, total_work)

        if not parts:
            raise RuntimeError(
                "Fusion export found no accessible solid bodies. Check the design's "
                "external component references and try again.")

        if progress_dialog:
            try:
                progress_dialog.progressValue = max(1, total_work)
            except Exception:
                pass
        ui.statusMessage = ""
        if progress_dialog:
            try:
                progress_dialog.hide()
            except Exception:
                pass
            progress_dialog = None

        manifest = {
            "schemaVersion": 2,
            "designName": design.rootComponent.name,
            "lengthUnit": units,
            "unitToMeters": unit_to_meters,
            "weaponMaxRpm": 30000.0,
            "weaponSpinUpSeconds": 1.5,
            "nonSolidSurfaceBodiesFound": total_surface_count,
            "nonSolidSurfaceBodiesSkipped": skipped_surface_count,
            "parts": parts,
        }
        manifest_path = os.path.join(output_folder, "manifest.json")
        with open(manifest_path, "w", encoding="utf-8") as manifest_file:
            json.dump(manifest, manifest_file, indent=2, ensure_ascii=False)

        warning_path = os.path.join(output_folder, "export_warnings.txt")
        if skipped_items:
            with open(warning_path, "w", encoding="utf-8") as warning_file:
                warning_file.write(
                    "Some Fusion components or bodies were unavailable and skipped. "
                    "Relink missing external references and re-export if those parts "
                    "are required.\n\n")
                warning_file.write("\n".join(skipped_items))
            preview_warnings = "\n".join(skipped_items[:8])
            if len(skipped_items) > 8:
                preview_warnings += "\n... {} more; see export_warnings.txt".format(
                    len(skipped_items) - 8)
            warning_summary = (
                "\n\nUnavailable components/bodies skipped: " +
                str(len(skipped_items)) + "\n" + preview_warnings +
                "\nFull details: " + warning_path
            )
        else:
            warning_summary = ""
            if os.path.isfile(warning_path):
                os.remove(warning_path)

        ui.messageBox(
            "CombatSim Fusion export finished.\n\n" + output_folder +
            "\n\nParts exported: " + str(len(parts)) +
            "\nB-Rep solids/surfaces: " + str(sum(
                part.get("geometryType") != "meshBody" for part in parts)) +
            "\nFusion mesh bodies: " + str(sum(
                part.get("geometryType") == "meshBody" for part in parts)) +
            "\nNon-solid surface bodies skipped: " + str(skipped_surface_count) +
            "\nImport manifest.json from the CombatSim Hub." + warning_summary
        )
    except:
        if progress_dialog:
            try:
                progress_dialog.hide()
            except Exception:
                pass
        if ui:
            ui.messageBox("CombatSim export failed:\n" + traceback.format_exc())
    finally:
        if progress_dialog:
            try:
                progress_dialog.hide()
            except Exception:
                pass
        if ui:
            try:
                ui.statusMessage = ""
            except Exception:
                pass
