import adsk.core
import adsk.fusion
import adsk.cam
import math
import os
import time
import traceback

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
VFOV_DEG = 150.0  # vertical field-of-view in degrees

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
    for ln in lines:
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


def switch_to_render_workspace() -> bool:
    app = adsk.core.Application.get()
    ui = app.userInterface
    ws = ui.workspaces.itemById("FusionRenderEnvironment")
    if ws:
        ws.activate()
        adsk.doEvents()
        return True
    return False


def setup_render_settings(width: int, height: int) -> adsk.fusion.Rendering:
    """
    Configure rendering settings using the design's Render Manager.
    Returns an adsk.fusion.Rendering object.
    """
    app = adsk.core.Application.get()
    design = adsk.fusion.Design.cast(app.activeProduct)
    if not design:
        raise RuntimeError("No active Fusion design.")

    render_mgr = design.renderManager
    rendering = render_mgr.rendering

    # Custom aspect ratio and resolution
    rendering.aspectRatio = adsk.fusion.RenderAspectRatios.CustomRenderAspectRatio
    rendering.resolutionWidth = int(width)
    rendering.resolutionHeight = int(height)

    # Quality (0..100); 100 = Excellent
    rendering.renderQuality = 100
    return rendering


def start_and_wait_local_render(
    rendering: adsk.fusion.Rendering,
    camera: adsk.core.Camera,
    filename: str,
    timeout_s: float = 300.0,
) -> None:
    """
    Perform a local render using the given camera and save to filename.
    Blocks until finished or raises on failure/timeout.
    """
    # Overwrite existing file to avoid any dialogs
    try:
        if os.path.exists(filename):
            os.remove(filename)
    except:
        pass

    render_future = rendering.startLocalRender(filename, camera)

    t0 = time.time()
    while True:
        state = render_future.renderState
        if state == adsk.fusion.LocalRenderStates.FinishedLocalRenderState:
            break
        if state == adsk.fusion.LocalRenderStates.FailedLocalRenderState:
            raise RuntimeError("Local render failed.")
        if time.time() - t0 > timeout_s:
            raise RuntimeError("Local render timed out.")
        adsk.doEvents()
        time.sleep(0.1)


def set_camera_and_render(
    app: adsk.core.Application,
    eye_pt_cm: adsk.core.Point3D,
    fwd_vec3: adsk.core.Vector3D,
    v_fov_deg: float,
    out_path: str,
):
    vp = app.activeViewport
    cam = vp.camera
    cam.isSmoothTransition = False
    cam.cameraType = adsk.core.CameraTypes.PerspectiveCameraType

    # Clamp FOV into a safe range and set as vertical FOV
    safe_fov = max(1.0, min(150.0, float(v_fov_deg)))
    cam.viewAngle = math.radians(safe_fov)

    # Eye and target (use fwd vector scaled to 100 cm)
    cam.eye = eye_pt_cm
    target = adsk.core.Point3D.create(
        eye_pt_cm.x + fwd_vec3.x * 100.0,
        eye_pt_cm.y + fwd_vec3.y * 100.0,
        eye_pt_cm.z + fwd_vec3.z * 100.0,
    )
    cam.target = target
    cam.upVector = adsk.core.Vector3D.create(0, 0, 1)
    cam.isFitView = False

    vp.camera = cam
    adsk.doEvents()

    # Configure rendering and run local render
    rendering = setup_render_settings(IMG_W, IMG_H)
    start_and_wait_local_render(rendering, vp.camera, out_path)


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

        # Switch to Render workspace
        switch_to_render_workspace()

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
        # Use NE, SE, SW, NW at corners
        dirs_diagonal = [
            ("NE", ne2),
            ("SE", se2),
            ("SW", sw2),
            ("NW", nw2),
        ]

        # Capture for each requested grid position
        for (gx, gy) in positions:
            # Center position (cm)
            mx_in, my_in, _ = grid_to_model_in(gx, gy)
            cx = inches_to_cm(mx_in)
            cy = inches_to_cm(my_in)
            cz = inches_to_cm(EYE_Z_IN)

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

                # Left eye render
                left_eye_pt = adsk.core.Point3D.create(
                    left_eye_xy[0], left_eye_xy[1], cz
                )
                left_file = os.path.join(
                    out_dir, f"{prefix}_{gx}_{gy}_{dir_name}_left.png"
                )
                set_camera_and_render(
                    app, left_eye_pt, left_fwd3, VFOV_DEG, left_file
                )

                # Right eye render
                right_eye_pt = adsk.core.Point3D.create(
                    right_eye_xy[0], right_eye_xy[1], cz
                )
                right_file = os.path.join(
                    out_dir, f"{prefix}_{gx}_{gy}_{dir_name}_right.png"
                )
                set_camera_and_render(
                    app, right_eye_pt, right_fwd3, VFOV_DEG, right_file
                )

        if ui:
            ui.messageBox(
                f"Capture completed. Images saved to:\n"
                f"{os.path.join(photos_dir, prefix)}"
            )

    except Exception as e:
        if ui:
            ui.messageBox("Failed:\n{}".format(traceback.format_exc()))
        else:
            print("Failed:\n{}".format(traceback.format_exc()))


def stop(context):
    # No persistent UI to clean up in this simple script
    pass
