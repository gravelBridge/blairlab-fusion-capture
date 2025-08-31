# capture.py
# Fusion 360 Python script: capture 128x128 stereo "rodent eye" renders from
# grid positions read from positions.txt.
#
# File structure:
#   photos/
#   capture.py
#   positions.txt
#
# positions.txt format:
#   file_prefix
#   x,y
#   x,y
#   ...
#
# Notes:
# - Grid (gx, gy) maps to model (X, Y) with axes swapped:
#     model X = f(gy), model Y = g(gx), Z fixed = 33.577 in
# - North is defined as direction from (5,5) to (5,0).
# - For corner cells (0,0), (10,0), (0,10), (10,10), use NE, SE, SW, NW.
# - Each grid position captures 8 images (4 directions per eye).
# - Eye separation: 0.5 in (±0.25 in from center along base right vector).
# - Eye yaw offset: ±50° from base direction; pitch up 15°; vertical FOV 150°.
# - Images saved under photos/<file_prefix>/ as PNG files and will overwrite.

import adsk.core
import adsk.fusion
import adsk.cam
import math
import os
import traceback


IN_PER_CM = 1.0 / 2.54  # not used directly; we convert inches -> cm
CM_PER_IN = 2.54

# Height of grid plane and eye elevation (inches)
GRID_Z_IN = 33.577
EYE_RAISE_IN = 2.5
EYE_Z_IN = GRID_Z_IN + EYE_RAISE_IN

# Eye and camera parameters
EYE_SEPARATION_IN = 0.5  # total separation
HALF_BASE_IN = EYE_SEPARATION_IN / 2.0
YAW_OFFSET_DEG = 50.0
PITCH_UP_DEG = 15.0
VFOV_DEG = 150.0

# Output image size
IMG_W = 128
IMG_H = 128

# Mapping from grid indices to model coordinates (inches)
# Axes are swapped per provided data:
#   model X comes from grid y; model Y comes from grid x
X_FROM_GRID_Y_IN = {
    0: 8.039,
    1: 16.336,
    5: 55.519,
    9: 94.702,
    10: 102.999,
}
Y_FROM_GRID_X_IN = {
    0: 6.02,
    1: 14.317,
    5: 53.50,
    9: 92.683,
    10: 100.98,
}


def _interp_linear(index: int, knot_map: dict) -> float:
    if index in knot_map:
        return knot_map[index]
    keys = sorted(knot_map.keys())
    if index <= keys[0]:
        return knot_map[keys[0]]
    if index >= keys[-1]:
        return knot_map[keys[-1]]
    k0 = None
    k1 = None
    for i in range(len(keys) - 1):
        if keys[i] <= index <= keys[i + 1]:
            k0 = keys[i]
            k1 = keys[i + 1]
            break
    if k0 is None or k1 is None or k1 == k0:
        return knot_map[keys[0]]
    t = (index - k0) / float(k1 - k0)
    return knot_map[k0] * (1.0 - t) + knot_map[k1] * t


def grid_to_model_in(gx: int, gy: int):
    # inches
    mx = _interp_linear(gy, X_FROM_GRID_Y_IN)  # swap axes
    my = _interp_linear(gx, Y_FROM_GRID_X_IN)
    mz = GRID_Z_IN
    return mx, my, mz


def inches_to_cm(x_in: float) -> float:
    return x_in * CM_PER_IN


def rot2d(vx: float, vy: float, deg: float):
    rad = math.radians(deg)
    c = math.cos(rad)
    s = math.sin(rad)
    return (vx * c - vy * s, vx * s + vy * c)


def norm2d(vx: float, vy: float):
    mag = math.hypot(vx, vy)
    if mag <= 1e-9:
        return (0.0, 0.0)
    return (vx / mag, vy / mag)


def add2d(a, b):
    return (a[0] + b[0], a[1] + b[1])


def scale2d(v, s):
    return (v[0] * s, v[1] * s)


def is_corner(gx: int, gy: int) -> bool:
    return (gx in (0, 10)) and (gy in (0, 10))


def ensure_dir(path: str):
    os.makedirs(path, exist_ok=True)


def parse_positions_txt(script_dir: str):
    path = os.path.join(script_dir, "positions.txt")
    prefix = None
    positions = []
    with open(path, "r", encoding="utf-8") as f:
        lines = [ln.strip() for ln in f.readlines()]
    for i, ln in enumerate(lines):
        if not ln or ln.startswith("#"):
            continue
        if prefix is None:
            prefix = ln
            continue
        parts = ln.split(",")
        if len(parts) != 2:
            continue
        try:
            gx = int(parts[0].strip())
            gy = int(parts[1].strip())
            positions.append((gx, gy))
        except:
            pass
    if prefix is None:
        raise RuntimeError("positions.txt missing file_prefix on first line.")
    return prefix, positions


def find_render_workspace(ui: adsk.core.UserInterface):
    # Try common ids; return the first that exists
    candidates = [
        "FusionRenderEnvironment",
        "RenderEnvironment",
        "RenderWorkspace",
        "Render",
    ]
    wss = ui.workspaces
    for wid in candidates:
        ws = wss.itemById(wid)
        if ws:
            return ws
    # Fallback: stay in current workspace
    return None


def set_camera_and_capture(
    app: adsk.core.Application,
    eye_pt_cm: adsk.core.Point3D,
    fwd_vec3: adsk.core.Vector3D,
    v_fov_deg: float,
    out_path: str,
):
    vp = app.activeViewport
    cam = vp.camera

    # Configure perspective camera with specified vertical FOV
    cam.isPerspective = True
    # Some APIs use 'perspectiveAngle', others 'viewAngle'
    # Fusion 360 uses perspectiveAngle (radians) for perspective FOV.
    try:
        cam.perspectiveAngle = math.radians(v_fov_deg)
    except:
        # Fallback if property differs (rare)
        try:
            cam.viewAngle = math.radians(v_fov_deg)
        except:
            pass

    cam.eye = eye_pt_cm
    # Target some distance along the forward vector
    fwd = fwd_vec3.copy()
    fwd.normalize()
    target = adsk.core.Point3D.create(
        eye_pt_cm.x + fwd.x * 10.0,
        eye_pt_cm.y + fwd.y * 10.0,
        eye_pt_cm.z + fwd.z * 10.0,
    )
    cam.target = target

    # Keep a world-up vector (no roll)
    cam.upVector = adsk.core.Vector3D.create(0, 0, 1)

    vp.camera = cam
    vp.refresh()
    vp.saveAsImageFile(out_path, IMG_W, IMG_H)


def run(context):
    app = adsk.core.Application.get()
    ui = app.userInterface if app else None
    try:
        script_dir = os.path.dirname(os.path.realpath(__file__))
        photos_dir = os.path.join(script_dir, "photos")
        ensure_dir(photos_dir)

        prefix, positions = parse_positions_txt(script_dir)
        out_dir = os.path.join(photos_dir, prefix)
        ensure_dir(out_dir)

        # Switch to Render workspace if available
        render_ws = find_render_workspace(ui)
        if render_ws:
            render_ws.activate()

        # Precompute base direction vectors using provided definition:
        # North = (5,5) -> (5,0), East = (5,5) -> (10,5)
        def model_xy_cm(gx, gy):
            mx_in, my_in, _ = grid_to_model_in(gx, gy)
            return inches_to_cm(mx_in), inches_to_cm(my_in)

        p_55 = model_xy_cm(5, 5)
        p_50 = model_xy_cm(5, 0)
        p_105 = model_xy_cm(10, 5)

        north2 = (p_50[0] - p_55[0], p_50[1] - p_55[1])
        east2 = (p_105[0] - p_55[0], p_105[1] - p_55[1])

        north2 = norm2d(*north2)
        east2 = norm2d(*east2)
        south2 = (-north2[0], -north2[1])
        west2 = (-east2[0], -east2[1])

        # Diagonals (normalized)
        ne2 = norm2d(*(north2[0] + east2[0], north2[1] + east2[1]))
        se2 = norm2d(*(south2[0] + east2[0], south2[1] + east2[1]))
        sw2 = norm2d(*(south2[0] + west2[0], south2[1] + west2[1]))
        nw2 = norm2d(*(north2[0] + west2[0], north2[1] + west2[1]))

        dirs_cardinal = [
            ("north", north2),
            ("east", east2),
            ("south", south2),
            ("west", west2),
        ]
        dirs_diagonal = [
            ("ne", ne2),
            ("se", se2),
            ("sw", sw2),
            ("nw", nw2),
        ]

        # Capture for each requested grid position
        for (gx, gy) in positions:
            # Center position (cm)
            mx_in, my_in, _ = grid_to_model_in(gx, gy)
            cx = inches_to_cm(mx_in)
            cy = inches_to_cm(my_in)
            cz = inches_to_cm(EYE_Z_IN)

            center_pt = adsk.core.Point3D.create(cx, cy, cz)

            # Which set of directions?
            dirs = dirs_diagonal if is_corner(gx, gy) else dirs_cardinal

            for dir_name, base_fwd2 in dirs:
                # Base "right" vector in XY plane (perpendicular, pointing right)
                # For a forward vector (fx, fy), a right vector is (fy, -fx).
                right2 = (base_fwd2[1], -base_fwd2[0])
                # Eye positions offset ±0.25" along base-right
                left_eye_xy = (
                    cx - right2[0] * inches_to_cm(HALF_BASE_IN),
                    cy - right2[1] * inches_to_cm(HALF_BASE_IN),
                )
                right_eye_xy = (
                    cx + right2[0] * inches_to_cm(HALF_BASE_IN),
                    cy + right2[1] * inches_to_cm(HALF_BASE_IN),
                )

                # Eye orientations: yaw ±50° from base forward, pitch +15°
                # Left eye looks "left" (CCW +50°), right eye looks "right" (-50°)
                left_fwd2 = rot2d(base_fwd2[0], base_fwd2[1], +YAW_OFFSET_DEG)
                right_fwd2 = rot2d(base_fwd2[0], base_fwd2[1], -YAW_OFFSET_DEG)

                # Apply pitch: forward3D = (XY * cos(pitch), z = sin(pitch))
                cp = math.cos(math.radians(PITCH_UP_DEG))
                sp = math.sin(math.radians(PITCH_UP_DEG))

                left_fwd3 = adsk.core.Vector3D.create(
                    left_fwd2[0] * cp, left_fwd2[1] * cp, sp
                )
                right_fwd3 = adsk.core.Vector3D.create(
                    right_fwd2[0] * cp, right_fwd2[1] * cp, sp
                )

                # Left eye capture
                left_eye_pt = adsk.core.Point3D.create(
                    left_eye_xy[0], left_eye_xy[1], cz
                )
                left_file = os.path.join(
                    out_dir, f"{prefix}_{gx}_{gy}_{dir_name}_left.png"
                )
                set_camera_and_capture(
                    app, left_eye_pt, left_fwd3, VFOV_DEG, left_file
                )

                # Right eye capture
                right_eye_pt = adsk.core.Point3D.create(
                    right_eye_xy[0], right_eye_xy[1], cz
                )
                right_file = os.path.join(
                    out_dir, f"{prefix}_{gx}_{gy}_{dir_name}_right.png"
                )
                set_camera_and_capture(
                    app, right_eye_pt, right_fwd3, VFOV_DEG, right_file
                )

        if ui:
            ui.messageBox(
                f"Capture completed. Images saved to:\n{os.path.join(photos_dir, prefix)}"
            )

    except Exception as e:
        if ui:
            ui.messageBox("Failed:\n{}".format(traceback.format_exc()))
        else:
            print("Failed:\n{}".format(traceback.format_exc()))


def stop(context):
    # No persistent UI to clean up in this simple script
    pass
