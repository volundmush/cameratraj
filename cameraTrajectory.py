"""Camera trajectory animation using SO(3) and SE(3) interpolation.

This script keeps the assignment in plain Python. It imports the existing
rayTrace.py scene/renderer and adds Lie-group camera poses, keyframes,
trajectory visualization, comparison frames, and tangent-space perturbations.
"""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
from typing import Iterable, List, Tuple

import numpy as np
from PIL import Image, ImageDraw

from rayTrace import Camera, Renderer, clamp01, make_scene, mode_to_config


EPS = 1e-9


def normalize(v: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(v)
    if n < EPS:
        return v
    return v / n


def skew(w: np.ndarray) -> np.ndarray:
    return np.array(
        [
            [0.0, -w[2], w[1]],
            [w[2], 0.0, -w[0]],
            [-w[1], w[0], 0.0],
        ]
    )


def so3_exp(w: np.ndarray) -> np.ndarray:
    theta = np.linalg.norm(w)
    W = skew(w)
    if theta < 1e-8:
        return np.eye(3) + W + 0.5 * (W @ W)
    a = math.sin(theta) / theta
    b = (1.0 - math.cos(theta)) / (theta * theta)
    return np.eye(3) + a * W + b * (W @ W)


def so3_log(R: np.ndarray) -> np.ndarray:
    cos_theta = float(np.clip((np.trace(R) - 1.0) * 0.5, -1.0, 1.0))
    theta = math.acos(cos_theta)
    if theta < 1e-8:
        return np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]]) * 0.5
    return theta / (2.0 * math.sin(theta)) * np.array(
        [R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]]
    )


def left_jacobian_so3(w: np.ndarray) -> np.ndarray:
    theta = np.linalg.norm(w)
    W = skew(w)
    if theta < 1e-8:
        return np.eye(3) + 0.5 * W + (1.0 / 6.0) * (W @ W)
    a = (1.0 - math.cos(theta)) / (theta * theta)
    b = (theta - math.sin(theta)) / (theta * theta * theta)
    return np.eye(3) + a * W + b * (W @ W)


def left_jacobian_so3_inv(w: np.ndarray) -> np.ndarray:
    theta = np.linalg.norm(w)
    W = skew(w)
    if theta < 1e-8:
        return np.eye(3) - 0.5 * W + (1.0 / 12.0) * (W @ W)
    half = 0.5 * theta
    cot_half = 1.0 / math.tan(half)
    c = (1.0 - half * cot_half) / (theta * theta)
    return np.eye(3) - 0.5 * W + c * (W @ W)


def se3_exp(xi: np.ndarray) -> np.ndarray:
    w = xi[:3]
    v = xi[3:]
    R = so3_exp(w)
    t = left_jacobian_so3(w) @ v
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def se3_log(T: np.ndarray) -> np.ndarray:
    w = so3_log(T[:3, :3])
    v = left_jacobian_so3_inv(w) @ T[:3, 3]
    return np.concatenate([w, v])


def invert_pose(T: np.ndarray) -> np.ndarray:
    R = T[:3, :3]
    t = T[:3, 3]
    out = np.eye(4)
    out[:3, :3] = R.T
    out[:3, 3] = -(R.T @ t)
    return out


def pose_from_eye_target(eye: Iterable[float], target: Iterable[float], up_hint: Iterable[float] = (0.0, 1.0, 0.0)) -> np.ndarray:
    eye_v = np.array(eye, dtype=float)
    target_v = np.array(target, dtype=float)
    up_v = np.array(up_hint, dtype=float)
    forward = normalize(target_v - eye_v)
    right = normalize(np.cross(forward, up_v))
    up = normalize(np.cross(right, forward))

    T = np.eye(4)
    T[:3, 0] = right
    T[:3, 1] = up
    T[:3, 2] = -forward
    T[:3, 3] = eye_v
    return T


def camera_from_pose(T: np.ndarray, aspect: float, fov: float = 50.0) -> Camera:
    eye = T[:3, 3]
    forward = T[:3, :3] @ np.array([0.0, 0.0, -1.0])
    up = T[:3, :3] @ np.array([0.0, 1.0, 0.0])
    return Camera(eye, eye + forward, up, fov_degrees=fov, aspect=aspect)


def keyframes() -> List[np.ndarray]:
    target = np.array([0.0, 0.05, -5.45])
    eyes = [
        (-3.0, 0.75, 0.9),
        (-1.4, 1.8, -0.4),
        (1.6, 1.25, -0.2),
        (3.0, 0.65, 1.0),
        (1.2, 2.35, 1.6),
        (-3.0, 0.75, 0.9),
    ]
    return [pose_from_eye_target(eye, target) for eye in eyes]


def segment_sample(poses: List[np.ndarray], u: float) -> Tuple[np.ndarray, np.ndarray, float, int]:
    count = len(poses) - 1
    scaled = min(max(u, 0.0), 1.0) * count
    seg = min(int(scaled), count - 1)
    local = scaled - seg
    return poses[seg], poses[seg + 1], local, seg


def smoothstep(t: float) -> float:
    return t * t * (3.0 - 2.0 * t)


def interpolate_se3(poses: List[np.ndarray], u: float) -> np.ndarray:
    A, B, s, _ = segment_sample(poses, u)
    s = smoothstep(s)
    return A @ se3_exp(s * se3_log(invert_pose(A) @ B))


def interpolate_so3(poses: List[np.ndarray], u: float) -> np.ndarray:
    A, B, s, _ = segment_sample(poses, u)
    s = smoothstep(s)
    R = A[:3, :3] @ so3_exp(s * so3_log(A[:3, :3].T @ B[:3, :3]))
    t = (1.0 - s) * A[:3, 3] + s * B[:3, 3]
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def interpolate_euclidean(poses: List[np.ndarray], u: float) -> np.ndarray:
    A, B, s, _ = segment_sample(poses, u)
    s = smoothstep(s)
    # Deliberately simple baseline: blend matrix entries, then re-orthonormalize.
    R_raw = (1.0 - s) * A[:3, :3] + s * B[:3, :3]
    u, _, vt = np.linalg.svd(R_raw)
    q = u @ vt
    if np.linalg.det(q) < 0.0:
        u[:, -1] *= -1.0
        q = u @ vt
    T = np.eye(4)
    T[:3, :3] = q
    T[:3, 3] = (1.0 - s) * A[:3, 3] + s * B[:3, 3]
    return T


def perturb_pose(T: np.ndarray, u: float, side: str) -> np.ndarray:
    wobble = math.sin(2.0 * math.pi * u)
    xi = np.array([0.05 * wobble, 0.18 * wobble, 0.03 * math.cos(4.0 * math.pi * u), 0.18 * wobble, 0.07 * math.cos(2.0 * math.pi * u), 0.0])
    dT = se3_exp(xi)
    if side == "left":
        return dT @ T
    if side == "right":
        return T @ dT
    raise ValueError(f"Unsupported perturbation side: {side}")


def pose_at(method: str, frame: int, frames: int) -> np.ndarray:
    poses = keyframes()
    u = 0.0 if frames <= 1 else frame / float(frames - 1)
    if method == "se3":
        return interpolate_se3(poses, u)
    if method == "so3":
        return interpolate_so3(poses, u)
    if method == "euclidean":
        return interpolate_euclidean(poses, u)
    if method == "left-perturbed":
        return perturb_pose(interpolate_se3(poses, u), u, "left")
    if method == "right-perturbed":
        return perturb_pose(interpolate_se3(poses, u), u, "right")
    raise ValueError(f"Unsupported method: {method}")


def trajectory(method: str, frames: int) -> List[np.ndarray]:
    return [pose_at(method, i, frames) for i in range(frames)]


def render_frame(args: argparse.Namespace) -> Path:
    out_dir = Path(args.output)
    out_dir.mkdir(parents=True, exist_ok=True)
    cfg = mode_to_config(args.mode, args.width, args.height)
    rng = np.random.default_rng(args.seed + args.frame)
    renderer = Renderer(make_scene(), cfg, rng)
    cam = camera_from_pose(pose_at(args.method, args.frame, args.frames), args.width / float(args.height))
    image = renderer.render(cam)
    out = (clamp01(image) * 255.0).astype(np.uint8)
    path = out_dir / f"{args.method}_{args.frame:04d}.png"
    Image.fromarray(out).save(path)
    return path


def render_all(args: argparse.Namespace) -> None:
    for frame in range(args.frames):
        args.frame = frame
        path = render_frame(args)
        print(path)


def write_csv(methods: List[str], frames: int, output: Path) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["method", "frame", "x", "y", "z", "forward_x", "forward_y", "forward_z"])
        for method in methods:
            for i, T in enumerate(trajectory(method, frames)):
                forward = T[:3, :3] @ np.array([0.0, 0.0, -1.0])
                writer.writerow([method, i, *T[:3, 3], *forward])


def draw_polyline(draw: ImageDraw.ImageDraw, pts: List[Tuple[float, float]], fill: Tuple[int, int, int], width: int) -> None:
    for a, b in zip(pts, pts[1:]):
        draw.line([a, b], fill=fill, width=width)


def visualize(methods: List[str], frames: int, output: Path) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    poses_by_method = {m: trajectory(m, frames) for m in methods}
    points = np.array([T[:3, 3] for poses in poses_by_method.values() for T in poses])
    min_x, max_x = float(points[:, 0].min()), float(points[:, 0].max())
    min_z, max_z = float(points[:, 2].min()), float(points[:, 2].max())
    pad = 0.65
    width, height = 1200, 800

    def project(p: np.ndarray) -> Tuple[float, float]:
        x = (p[0] - min_x + pad) / (max_x - min_x + 2.0 * pad) * width
        y = height - (p[2] - min_z + pad) / (max_z - min_z + 2.0 * pad) * height
        return x, y

    colors = {
        "se3": (230, 70, 55),
        "so3": (60, 130, 230),
        "euclidean": (90, 90, 90),
        "left-perturbed": (60, 170, 95),
        "right-perturbed": (165, 85, 220),
    }
    img = Image.new("RGB", (width, height), (248, 246, 240))
    draw = ImageDraw.Draw(img)
    draw.text((24, 22), "Camera center paths, top-down X/Z view", fill=(30, 30, 30))
    draw.text((24, 46), "Small arrows show camera forward direction", fill=(70, 70, 70))

    for idx, (method, poses) in enumerate(poses_by_method.items()):
        color = colors[method]
        pts = [project(T[:3, 3]) for T in poses]
        draw_polyline(draw, pts, color, 4 if method == "se3" else 2)
        draw.text((24, 82 + idx * 22), method, fill=color)
        step = max(1, frames // 12)
        for T in poses[::step]:
            p = project(T[:3, 3])
            fwd = T[:3, :3] @ np.array([0.0, 0.0, -1.0])
            q = project(T[:3, 3] + 0.28 * normalize(fwd))
            draw.line([p, q], fill=color, width=2)
            draw.ellipse((p[0] - 3, p[1] - 3, p[0] + 3, p[1] + 3), fill=color)

    target = project(np.array([0.0, 0.0, -5.45]))
    draw.ellipse((target[0] - 7, target[1] - 7, target[0] + 7, target[1] + 7), outline=(0, 0, 0), width=2)
    draw.text((target[0] + 10, target[1] - 8), "scene focus", fill=(0, 0, 0))
    img.save(output)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="SE(3) camera trajectory renderer")
    parser.add_argument("--method", default="se3", choices=["se3", "so3", "euclidean", "left-perturbed", "right-perturbed"])
    parser.add_argument("--mode", default="all", choices=["basic", "reflection", "refraction", "glossy", "softshadow", "all"])
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=360)
    parser.add_argument("--frames", type=int, default=120)
    parser.add_argument("--frame", type=int, default=0)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--output", default="frames")
    parser.add_argument("--render-all", action="store_true", help="Render every frame sequentially")
    parser.add_argument("--visualize", action="store_true", help="Write trajectory visualization and CSV, then exit")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    methods = ["se3", "so3", "euclidean", "left-perturbed", "right-perturbed"]
    if args.visualize:
        visualize(methods, args.frames, Path("outputs/trajectory_comparison.png"))
        write_csv(methods, args.frames, Path("outputs/trajectory_samples.csv"))
        print("outputs/trajectory_comparison.png")
        print("outputs/trajectory_samples.csv")
        return
    if args.render_all:
        render_all(args)
        return
    print(render_frame(args))


if __name__ == "__main__":
    main()
