# Camera Trajectory Animation Using Lie-Group Interpolation

This project extends the previous plain-Python ray tracer with a moving camera animation. The camera path is built from several keyframe poses and interpolated on Lie groups instead of by treating camera matrices like ordinary numbers.

The deliverable code is script-based: no notebooks are required. This is because I am struggling to handle advanced operations in Notebooks that use complex system libraries while using Bazzite as my OS, and those libraries also aren't liking my current outdated GPU one bit.

## How To Run

Install dependencies:

```bash
python3 -m pip install -r requirements.txt
```

Generate the trajectory plot and sampled camera data:

```bash
python3 cameraTrajectory.py --visualize --frames 120
```

Render one animation frame:

```bash
python3 cameraTrajectory.py --method se3 --frame 0 --frames 120 --width 640 --height 360 --output frames
```

Render the full SE(3) animation using the CPU-parallel shell script:

```bash
./generateImages.sh animation 120 640 360
```

If `ffmpeg` is installed, the script also writes:

```text
outputs/se3_camera_trajectory.mp4
```

Otherwise, the rendered PNG sequence is in `frames/` and can be encoded later.

The previous still-image assignment renders are still available:

```bash
./generateImages.sh stills 1 1280 720
```

## Important Files

`rayTrace.py` is the original recursive ray tracer. It still owns the scene, materials, camera rays, reflections, refractions, glossy reflection, and soft shadows.

`cameraTrajectory.py` adds the new moving camera:

- SE(3) exponential and logarithm maps
- SO(3) exponential and logarithm maps
- several camera keyframes
- geodesic interpolation between poses
- Euclidean baseline interpolation
- tangent-space left and right perturbation experiments
- frame rendering through the existing ray tracer
- trajectory visualization and CSV export

`generateImages.sh` parallelizes frame rendering across CPU cores and optionally encodes the result with `ffmpeg`.

`outputs/trajectory_comparison.png` is the required trajectory visualization.

`outputs/trajectory_samples.csv` contains sampled camera positions and forward vectors for each method.

## Scene And Animation Result

The rendered scene contains three spheres over a floor plane: a red glossy reflective sphere, a green matte sphere, and a glass sphere with refraction. Soft shadows and recursive bounces are enabled when rendering with `--mode all`.

The main animation uses the `se3` method. The camera moves in a closed loop around the scene, rises over the objects, returns to the starting point, and keeps its orientation tied to the same rigid camera pose representation throughout the motion.

## What Is A Camera Pose?

A camera pose answers two questions:

- Where is the camera?
- Which way is it facing?

In this project, a pose is a 4 by 4 rigid transform:

```text
T = [ R  t ]
    [ 0  1 ]
```

`R` is a 3 by 3 rotation matrix. It stores the camera's right, up, and backward directions in world space. `t` is the camera center in world space.

Plain English version: `T` is a compact box that says, "put the camera here, turn it this way, and do not stretch or shear it."

## Why Use Lie Groups?

Camera poses do not live in normal flat space. A valid camera rotation must stay orthonormal, meaning its axes remain perpendicular and unit length. If we just average rotation matrices entry by entry, the result can become an invalid or distorted rotation.

The set of 3D rotations is called SO(3), the Special Orthogonal Group. The set of 3D rigid motions, meaning rotation plus translation, is called SE(3), the Special Euclidean Group.

Plain English version: SO(3) is the space of legal turns. SE(3) is the space of legal turns plus legal moves. Lie-group interpolation moves through those legal spaces instead of cutting through invalid poses.

## Exponential And Logarithm Maps

The code uses two maps:

```text
exp: tangent vector -> pose
log: pose -> tangent vector
```

The tangent vector is called a twist. It has six numbers for SE(3):

```text
xi = [ wx wy wz vx vy vz ]
```

`w` describes angular motion. `v` describes translational motion. The exponential map turns that small motion instruction into a full rigid transform. The logarithm map does the reverse.

Plain English version: `log` converts a pose difference into an instruction like "rotate this much and move this much." `exp` follows that instruction to produce a new pose.

## SE(3) Interpolation Method

For two keyframes `A` and `B`, the SE(3) interpolation is:

```text
T(s) = A exp(s log(A^-1 B))
```

where `s` goes from 0 to 1.

What this does:

- `A^-1 B` finds the motion from keyframe `A` to keyframe `B`.
- `log(A^-1 B)` turns that motion into a twist.
- Multiplying the twist by `s` takes only part of the motion.
- `exp(...)` turns that partial motion back into a rigid transform.
- Multiplying by `A` places the partial motion back in world space.

Plain English version: find the exact rigid motion from camera A to camera B, then follow 0%, 1%, 2%, and so on of that motion. Every intermediate camera is still a real rigid camera pose.

The code also applies `smoothstep(s) = s^2(3 - 2s)` inside each segment. That eases the camera in and out of keyframes so the animation looks less robotic.

## SO(3) Versus SE(3) Bonus

The `so3` method interpolates rotation on SO(3), but translates the camera center with ordinary linear interpolation:

```text
R(s) = R_A exp(s log(R_A^T R_B))
t(s) = (1 - s)t_A + s t_B
```

This is still geometrically valid for orientation because the camera rotation stays on SO(3). However, translation is not coupled to rotation the way it is in full SE(3).

Plain English version: SO(3) interpolation turns the camera properly, but moves the camera position like a dot sliding along a line. SE(3) treats the turn and move as one combined rigid-body action.

## Euclidean Baseline Comparison

The `euclidean` method intentionally uses a simpler baseline. It blends rotation matrix entries and camera centers, then re-orthonormalizes the rotation matrix with QR decomposition so the result can still be rendered.

This is useful as a comparison, not as the preferred method.

Plain English version: Euclidean interpolation says, "average the numbers and clean up the mess afterward." Lie-group interpolation says, "move through the space where legal camera poses already live."

Observed behavior:

- SE(3) gives consistent rigid-body motion between keyframes.
- SO(3) keeps rotation clean but does not couple translation and rotation.
- The Euclidean baseline can produce less natural orientation changes because the matrix blend is not the actual shortest rotation on the rotation manifold.

## Tangent-Space Perturbation Bonus

The script includes two perturbed SE(3) paths:

- `left-perturbed`
- `right-perturbed`

Both start with the SE(3) trajectory and apply a small twist. The twist is sinusoidal, so the camera gently wobbles instead of jumping randomly.

Plain English version: we take the smooth camera path and add a tiny controlled nudge in tangent space. Because the nudge is passed through `exp`, it remains a legal rigid transform.

## Left Versus Right Perturbations Bonus

Left perturbation uses:

```text
T' = exp(xi) T
```

Right perturbation uses:

```text
T' = T exp(xi)
```

They look similar on paper, but they behave differently.

Left perturbation applies the nudge in the world frame. If the nudge says "move right," it means world-right.

Right perturbation applies the nudge in the camera's local frame. If the nudge says "move right," it means the camera's own right direction, which changes as the camera turns.

Plain English version: left perturbation is like pushing the camera from the room's point of view. Right perturbation is like pushing the camera from the camera operator's point of view.

## Complex Closed-Loop Trajectory Bonus

The keyframe list contains six poses, with the final pose equal to the first pose. This makes the camera path a closed loop. The camera moves around the scene, changes height, changes viewing angle, and returns to the start so the animation can loop cleanly.

Plain English version: this is not just camera A to camera B. It is a multi-stop camera move that comes back home.

## Implementation Notes

The math is implemented in `cameraTrajectory.py` using only `numpy`:

- `so3_exp` and `so3_log` implement Rodrigues-style rotation maps.
- `se3_exp` and `se3_log` use the SO(3) left Jacobian to handle coupled rotation and translation.
- `pose_from_eye_target` builds camera keyframes from eye and target points.
- `camera_from_pose` converts an SE(3) pose back into the existing `rayTrace.py` `Camera` class.
- `visualize` draws the trajectory plot using PIL, so no matplotlib dependency is needed.

## Deliverables Checklist

- Ray-traced animation: `./generateImages.sh animation 120 640 360`
- Trajectory definition with multiple keyframes: `cameraTrajectory.py`, `keyframes()`
- Lie-group interpolation: `interpolate_se3()` and `interpolate_so3()`
- Trajectory visualization: `outputs/trajectory_comparison.png`
- SE(3) versus SO(3) versus Euclidean comparison: `cameraTrajectory.py --visualize`
- Tangent-space perturbations: `left-perturbed` and `right-perturbed`
- Report: this `README.md` you're reading right now!

## Video
The output was small, so it's `demo.mp4` in the project root.