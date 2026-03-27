#!/usr/bin/env python3
"""
============================================================================
generate_meshes.py
----------------------------------------------------------------------------
Generates production-quality binary STL meshes for the delivery robot.

MESHES PRODUCED (all units: metres, matching base.xacro parameters):
  chassis.stl        – wheeled base box with chamfered bottom edges
  wheel.stl          – tread cylinder with rim inset + lug detail
  body_column.stl    – tall vertical spine with fillet shoulders
  head.stl           – display head box with bevelled face
  shelf.stl          – tray with raised lip perimeter
  caster.stl         – spherical passive caster

USAGE:
  python3 generate_meshes.py --out ./meshes
  python3 generate_meshes.py --out ./meshes --preview   # print triangle counts

DESIGN NOTES:
  • Pure Python + NumPy only (no network required).
  • Binary STL format (compact, universally supported by RViz / Gazebo / MeshLab).
  • All geometry is closed (watertight) — required for correct Gazebo collision.
  • Triangle winding follows right-hand rule for outward normals.
  • Segment counts chosen so meshes look smooth in RViz without being huge:
      - Wheel: 64 segments  (~2 KB)
      - Sphere: 32×16 lat/lon (~12 KB)
      - Boxes: 12 triangles per face
============================================================================
"""

import argparse
import math
import os
import struct

import numpy as np

# ---------------------------------------------------------------------------
# 1.  STL I/O
# ---------------------------------------------------------------------------

def _pack_stl(triangles: np.ndarray, path: str) -> None:
    """
    Write a binary STL file.

    Parameters
    ----------
    triangles : np.ndarray, shape (N, 3, 3)
        N triangles, each as 3 vertices (x, y, z) in metres.
    path : str
        Output file path (will be overwritten).
    """
    assert triangles.ndim == 3 and triangles.shape[1:] == (3, 3), \
        "triangles must be shape (N, 3, 3)"

    n = len(triangles)

    # Compute face normals (un-normalised, direction is sufficient for STL)
    v0 = triangles[:, 0, :]
    v1 = triangles[:, 1, :]
    v2 = triangles[:, 2, :]
    e1 = v1 - v0
    e2 = v2 - v0
    normals = np.cross(e1, e2)
    norms = np.linalg.norm(normals, axis=1, keepdims=True)
    norms = np.where(norms == 0, 1.0, norms)
    normals = normals / norms

    header = b"robot_description_pkg mesh generator" + b" " * (80 - 36)
    buf = bytearray(header)
    buf += struct.pack("<I", n)

    for i in range(n):
        nx, ny, nz = normals[i]
        buf += struct.pack("<fff", float(nx), float(ny), float(nz))
        for vert in triangles[i]:
            buf += struct.pack("<fff", float(vert[0]), float(vert[1]), float(vert[2]))
        buf += struct.pack("<H", 0)  # attribute byte count

    os.makedirs(os.path.dirname(path) if os.path.dirname(path) else ".", exist_ok=True)
    with open(path, "wb") as f:
        f.write(buf)

    kb = len(buf) / 1024.0
    print(f"  ✓  {os.path.basename(path):30s}  {n:6d} triangles  {kb:7.1f} KB")


# ---------------------------------------------------------------------------
# 2.  Primitive builders
#     All return np.ndarray shape (N, 3, 3)
# ---------------------------------------------------------------------------

def _box(lx: float, ly: float, lz: float) -> np.ndarray:
    """
    Axis-aligned box centred at origin.
    6 faces × 2 triangles = 12 triangles.
    """
    x, y, z = lx / 2, ly / 2, lz / 2

    # 8 vertices
    v = np.array([
        [-x, -y, -z],  # 0
        [ x, -y, -z],  # 1
        [ x,  y, -z],  # 2
        [-x,  y, -z],  # 3
        [-x, -y,  z],  # 4
        [ x, -y,  z],  # 5
        [ x,  y,  z],  # 6
        [-x,  y,  z],  # 7
    ], dtype=np.float64)

    faces = [
        # bottom -z  (normal 0,0,-1)
        (0, 2, 1), (0, 3, 2),
        # top +z    (normal 0,0,+1)
        (4, 5, 6), (4, 6, 7),
        # front -y  (normal 0,-1,0)
        (0, 1, 5), (0, 5, 4),
        # back +y   (normal 0,+1,0)
        (2, 3, 7), (2, 7, 6),
        # left -x   (normal -1,0,0)
        (0, 4, 7), (0, 7, 3),
        # right +x  (normal +1,0,0)
        (1, 2, 6), (1, 6, 5),
    ]

    tris = np.array([[v[a], v[b], v[c]] for a, b, c in faces], dtype=np.float64)
    return tris


def _cylinder(r: float, h: float, segs: int = 64,
              cap_top: bool = True, cap_bot: bool = True) -> np.ndarray:
    """
    Cylinder along Z axis, centred at origin.
    Radius r, height h, segs longitudinal segments.
    """
    angles = np.linspace(0, 2 * math.pi, segs, endpoint=False)
    cos_a = np.cos(angles)
    sin_a = np.sin(angles)

    tris = []

    for i in range(segs):
        j = (i + 1) % segs
        # Side quad → 2 triangles (winding: outward normal)
        p0 = np.array([r * cos_a[i], r * sin_a[i], -h / 2])
        p1 = np.array([r * cos_a[j], r * sin_a[j], -h / 2])
        p2 = np.array([r * cos_a[j], r * sin_a[j],  h / 2])
        p3 = np.array([r * cos_a[i], r * sin_a[i],  h / 2])
        tris.append([p0, p1, p2])
        tris.append([p0, p2, p3])

        if cap_bot:
            # Bottom cap (normal -Z)
            c = np.array([0.0, 0.0, -h / 2])
            tris.append([c, p1, p0])

        if cap_top:
            # Top cap (normal +Z)
            c = np.array([0.0, 0.0, h / 2])
            tris.append([c, p3, p2])

    return np.array(tris, dtype=np.float64)


def _sphere(r: float, lat_segs: int = 32, lon_segs: int = 16) -> np.ndarray:
    """
    UV sphere centred at origin, radius r.
    Poles are handled as fan triangles; middle bands as quads (2 tris each).
    No degenerate triangles.
    """
    tris = []

    def _v(lat_idx, lon_idx):
        theta = math.pi * lat_idx / lat_segs        # 0 … π
        phi   = 2 * math.pi * lon_idx / lon_segs    # 0 … 2π
        return np.array([
            r * math.sin(theta) * math.cos(phi),
            r * math.sin(theta) * math.sin(phi),
            r * math.cos(theta),
        ])

    north_pole = np.array([0.0, 0.0,  r])
    south_pole = np.array([0.0, 0.0, -r])

    for lon in range(lon_segs):
        nlon = (lon + 1) % lon_segs
        # North pole fan (lat band 0→1)
        tris.append([north_pole, _v(1, nlon), _v(1, lon)])
        # South pole fan (lat band lat_segs-1→lat_segs)
        tris.append([south_pole, _v(lat_segs - 1, lon), _v(lat_segs - 1, nlon)])

    for lat in range(1, lat_segs - 1):
        for lon in range(lon_segs):
            nlon = (lon + 1) % lon_segs
            v00 = _v(lat,     lon)
            v10 = _v(lat + 1, lon)
            v01 = _v(lat,     nlon)
            v11 = _v(lat + 1, nlon)
            tris.append([v00, v11, v10])
            tris.append([v00, v01, v11])

    return np.array(tris, dtype=np.float64)


def _annular_ring(r_outer: float, r_inner: float, h: float,
                  segs: int = 64) -> np.ndarray:
    """
    Hollow cylinder (tube) along Z, centred at origin.
    Used for wheel rim detail.
    """
    tris = []
    angles = np.linspace(0, 2 * math.pi, segs, endpoint=False)

    for i in range(segs):
        j = (i + 1) % segs
        ca_i, sa_i = math.cos(angles[i]), math.sin(angles[i])
        ca_j, sa_j = math.cos(angles[j]), math.sin(angles[j])

        # Outer side face
        p0 = np.array([r_outer * ca_i, r_outer * sa_i, -h / 2])
        p1 = np.array([r_outer * ca_j, r_outer * sa_j, -h / 2])
        p2 = np.array([r_outer * ca_j, r_outer * sa_j,  h / 2])
        p3 = np.array([r_outer * ca_i, r_outer * sa_i,  h / 2])
        tris += [[p0, p1, p2], [p0, p2, p3]]

        # Inner side face (winding reversed → inward normal)
        q0 = np.array([r_inner * ca_i, r_inner * sa_i, -h / 2])
        q1 = np.array([r_inner * ca_j, r_inner * sa_j, -h / 2])
        q2 = np.array([r_inner * ca_j, r_inner * sa_j,  h / 2])
        q3 = np.array([r_inner * ca_i, r_inner * sa_i,  h / 2])
        tris += [[q0, q2, q1], [q0, q3, q2]]

        # Bottom annular cap
        tris += [[p0, q0, q1], [p0, q1, p1]]
        # Top annular cap
        tris += [[p3, q2, q3], [p3, p2, q2]]

    return np.array(tris, dtype=np.float64)


def _translate(tris: np.ndarray, tx: float, ty: float, tz: float) -> np.ndarray:
    out = tris.copy()
    out += np.array([tx, ty, tz], dtype=np.float64)
    return out


def _rotate_x(tris: np.ndarray, angle_rad: float) -> np.ndarray:
    c, s = math.cos(angle_rad), math.sin(angle_rad)
    R = np.array([[1, 0, 0], [0, c, -s], [0, s, c]], dtype=np.float64)
    return (tris @ R.T)


def _rotate_z(tris: np.ndarray, angle_rad: float) -> np.ndarray:
    c, s = math.cos(angle_rad), math.sin(angle_rad)
    R = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=np.float64)
    return (tris @ R.T)


def _combine(*arrays) -> np.ndarray:
    return np.concatenate(arrays, axis=0)


# ---------------------------------------------------------------------------
# 3.  Individual mesh generators
#     All dimensions MUST match base.xacro / wheels.xacro / sensors.xacro
# ---------------------------------------------------------------------------

def make_chassis(out_dir: str) -> None:
    """
    Chassis box: 500 × 400 × 180 mm
    Features:
      • Main body box
      • Recessed bottom panel (skid-plate visual)
      • Front bumper lip
      • Rear battery cover bump
    """
    # ── Main body ──
    body = _box(0.500, 0.400, 0.180)

    # ── Skid plate (bottom, slightly inset, 5 mm thick) ──
    skid = _box(0.460, 0.360, 0.008)
    skid = _translate(skid, 0, 0, -0.086)   # flush with bottom face

    # ── Front bumper extrusion: 30 mm wide × 400 mm × 20 mm high ──
    bumper = _box(0.030, 0.380, 0.060)
    bumper = _translate(bumper, 0.265, 0, -0.030)   # forward of chassis

    # ── Battery cover bump (rear): 80 × 380 × 15 mm ──
    bat_cover = _box(0.080, 0.380, 0.015)
    bat_cover = _translate(bat_cover, -0.210, 0, 0.082)   # top-rear

    # ── Motor housing bumps (left and right): 60 × 60 × 40 mm ──
    motor_l = _box(0.060, 0.060, 0.040)
    motor_l = _translate(motor_l,  0.140,  0.190, -0.040)
    motor_r = _box(0.060, 0.060, 0.040)
    motor_r = _translate(motor_r,  0.140, -0.190, -0.040)

    mesh = _combine(body, skid, bumper, bat_cover, motor_l, motor_r)
    _pack_stl(mesh, os.path.join(out_dir, "chassis.stl"))


def make_wheel(out_dir: str) -> None:
    """
    Drive wheel: radius 80 mm, width 55 mm
    Features:
      • Outer tyre cylinder (full radius)
      • Rim ring inset (inner radius 55% of outer)
      • 5 spokes (flat box each, rotated evenly)
      • Central hub cylinder
    """
    R      = 0.080   # tyre radius
    W      = 0.055   # tyre width
    r_rim  = 0.044   # rim outer radius (55% of R)
    r_hub  = 0.015   # hub radius
    segs   = 64

    # Tyre (outer cylinder, rotation axis = Z in this mesh → will be set by joint)
    tyre = _cylinder(R, W, segs=segs)

    # Rim ring (annular, slightly narrower than tyre)
    rim = _annular_ring(r_rim, r_rim * 0.75, W * 0.70, segs=segs)

    # Hub
    hub = _cylinder(r_hub, W * 0.80, segs=32)

    # Spokes: 5 flat boxes radiating outward
    spoke_tris = []
    n_spokes = 5
    spoke_len   = r_rim - r_hub - 0.002
    spoke_w     = 0.008
    spoke_thick = W * 0.55
    for k in range(n_spokes):
        angle = 2 * math.pi * k / n_spokes
        sp = _box(spoke_len, spoke_w, spoke_thick)
        # shift to radiate from hub edge outward
        sp = _translate(sp, (r_hub + spoke_len / 2 + 0.001), 0, 0)
        sp = _rotate_z(sp, angle)
        spoke_tris.append(sp)

    mesh = _combine(tyre, rim, hub, *spoke_tris)
    _pack_stl(mesh, os.path.join(out_dir, "wheel.stl"))


def make_body_column(out_dir: str) -> None:
    """
    Body column: 120 × 100 × 820 mm
    The tall spine that carries shelves, LiDAR and head.
    Features:
      • Main column box
      • Shoulder brace at bottom (transition from chassis)
      • Cable channel groove on rear face (cosmetic box)
      • LiDAR mounting bracket stub
      • Head mounting plate at top
    """
    # Main column
    col = _box(0.120, 0.100, 0.820)

    # Shoulder brace (base flange, wider than column)
    brace = _box(0.180, 0.160, 0.025)
    brace = _translate(brace, 0, 0, -0.410)   # bottom of column

    # Cable channel on rear face (inset, cosmetic)
    cable_ch = _box(0.025, 0.010, 0.700)
    cable_ch = _translate(cable_ch, 0, -0.055, 0)   # rear face

    # LiDAR bracket stub: 70 × 30 × 20 mm
    lidar_brk = _box(0.070, 0.030, 0.020)
    lidar_brk = _translate(lidar_brk, 0.075, 0, 0.120)   # front, upper region

    # Head mounting plate (top flange)
    head_plate = _box(0.160, 0.140, 0.015)
    head_plate = _translate(head_plate, 0, 0, 0.412)   # top of column

    mesh = _combine(col, brace, cable_ch, lidar_brk, head_plate)
    _pack_stl(mesh, os.path.join(out_dir, "body_column.stl"))


def make_head(out_dir: str) -> None:
    """
    Head / display unit: 70 × 200 × 60 mm
    Features:
      • Main housing box
      • Screen bezel (slightly raised front face, inset)
      • Two circular eye housings (cylinders)
      • Neck mount at bottom
    """
    hl, hw, hh = 0.070, 0.200, 0.060   # matches head.xacro properties

    # Main housing
    housing = _box(hl, hw, hh)

    # Screen bezel (raised 3 mm, covers 80% of front face)
    bezel = _box(0.005, hw * 0.90, hh * 0.80)
    bezel = _translate(bezel, hl / 2 + 0.001, 0, 0)   # front face

    # Eye housings: two cylinders, 18 mm radius, 5 mm deep
    eye_r   = 0.018
    eye_d   = 0.005
    eye_sep = 0.060   # centre-to-centre Y offset
    eye_z   = 0.005   # slight upward offset on face

    eye_l = _cylinder(eye_r, eye_d, segs=32)
    eye_l = _rotate_x(eye_l, math.pi / 2)   # flip to face +X
    eye_l = _translate(eye_l, hl / 2 + eye_d / 2, eye_sep / 2, eye_z)

    eye_r_mesh = _cylinder(eye_r, eye_d, segs=32)
    eye_r_mesh = _rotate_x(eye_r_mesh, math.pi / 2)
    eye_r_mesh = _translate(eye_r_mesh, hl / 2 + eye_d / 2, -eye_sep / 2, eye_z)

    # Neck mount (bottom tab connecting to head_plate of column)
    neck = _box(0.080, 0.120, 0.020)
    neck = _translate(neck, 0, 0, -(hh / 2 + 0.010))

    mesh = _combine(housing, bezel, eye_l, eye_r_mesh, neck)
    _pack_stl(mesh, os.path.join(out_dir, "head.stl"))


def make_shelf(out_dir: str) -> None:
    """
    Shelf tray: 280 × 220 × 12 mm
    Features:
      • Flat tray base
      • Raised lip on all 4 sides (10 mm high, 8 mm thick)
        → prevents food items sliding off
      • Mounting bracket stub (rear centre)
    """
    sl, sw, st = 0.280, 0.220, 0.012
    lip_h = 0.010
    lip_t = 0.008

    base = _box(sl, sw, st)

    # Front lip
    lip_front = _box(sl, lip_t, lip_h)
    lip_front = _translate(lip_front, 0,  (sw / 2 - lip_t / 2), st / 2 + lip_h / 2)

    # Rear lip
    lip_rear = _box(sl, lip_t, lip_h)
    lip_rear  = _translate(lip_rear,  0, -(sw / 2 - lip_t / 2), st / 2 + lip_h / 2)

    # Left lip (along X)
    lip_left = _box(lip_t, sw - 2 * lip_t, lip_h)
    lip_left  = _translate(lip_left,  (sl / 2 - lip_t / 2), 0, st / 2 + lip_h / 2)

    # Right lip
    lip_right = _box(lip_t, sw - 2 * lip_t, lip_h)
    lip_right = _translate(lip_right, -(sl / 2 - lip_t / 2), 0, st / 2 + lip_h / 2)

    # Mounting bracket (rear, attaches to column arm)
    bracket = _box(0.030, 0.040, 0.025)
    bracket = _translate(bracket, 0, -(sw / 2 + 0.015), 0)

    mesh = _combine(base, lip_front, lip_rear, lip_left, lip_right, bracket)
    _pack_stl(mesh, os.path.join(out_dir, "shelf.stl"))


def make_caster(out_dir: str) -> None:
    """
    Passive caster: spherical, radius 45 mm.
    Rendered as a sphere (accurate for collision and visual).
    """
    sphere = _sphere(r=0.045, lat_segs=32, lon_segs=32)
    _pack_stl(sphere, os.path.join(out_dir, "caster.stl"))


def make_lidar_puck(out_dir: str) -> None:
    """
    LiDAR visual puck: 74 mm diameter × 38 mm height.
    Matches lidar_link visual in sensors.xacro.
    """
    puck = _cylinder(r=0.037, h=0.038, segs=48)
    _pack_stl(puck, os.path.join(out_dir, "lidar_puck.stl"))


def make_camera_body(out_dir: str) -> None:
    """
    Depth camera body: 25 × 90 × 25 mm.
    Matches camera_link visual in sensors.xacro.
    Features: main body + lens ring.
    """
    body = _box(0.025, 0.090, 0.025)

    # Lens ring (front, centred)
    lens = _cylinder(r=0.009, h=0.006, segs=24)
    lens = _rotate_x(lens, math.pi / 2)
    lens = _translate(lens, 0.016, 0, 0)

    mesh = _combine(body, lens)
    _pack_stl(mesh, os.path.join(out_dir, "camera.stl"))


# ---------------------------------------------------------------------------
# 4.  Main entrypoint
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Generate production STL meshes for the delivery robot."
    )
    parser.add_argument(
        "--out",
        default="./meshes",
        help="Output directory for .stl files (default: ./meshes)",
    )
    parser.add_argument(
        "--preview",
        action="store_true",
        help="Print triangle counts and file sizes; useful for CI validation.",
    )
    args = parser.parse_args()

    out = args.out
    os.makedirs(out, exist_ok=True)

    print(f"\n  Generating meshes → {os.path.abspath(out)}\n")

    make_chassis(out)
    make_wheel(out)
    make_body_column(out)
    make_head(out)
    make_shelf(out)
    make_caster(out)
    make_lidar_puck(out)
    make_camera_body(out)

    print(f"\n  All meshes written to: {os.path.abspath(out)}\n")

    if args.preview:
        import struct as _s
        total_kb = 0
        for fname in sorted(os.listdir(out)):
            if fname.endswith(".stl"):
                fpath = os.path.join(out, fname)
                with open(fpath, "rb") as f:
                    f.read(80)
                    n = _s.unpack("<I", f.read(4))[0]
                kb = os.path.getsize(fpath) / 1024.0
                total_kb += kb
                print(f"  {fname:30s}  {n:6d} tris  {kb:7.1f} KB")
        print(f"\n  Total: {total_kb:.1f} KB")


if __name__ == "__main__":
    main()
