# Assignment: Camera Trajectory Animation Using Lie-Group Interpolation

In this assignment, you will extend your previous ray tracing project by generating an **animation** of your scene as viewed from a **moving camera** following a continuous trajectory. Rather than specifying camera motion using standard Euclidean interpolation, you will construct and interpolate the camera trajectory using methods on Special Euclidean Group.

Your goal is to represent the camera poses as rigid transformations on the manifold, define a set of keyframe poses, and generate a smooth trajectory between them using Lie-group interpolation (for example, using exponential and logarithm maps, geodesic interpolation, or related manifold methods). The resulting interpolated poses will be used to render a sequence of frames that form a video animation of the scene.

You should use the provided notes and linked resources as a guide for:

- representing camera poses on Special Euclidean Group
- interpolating trajectories using Lie-group methods
- understanding tangent-space perturbations and geodesic motion
- comparing manifold interpolation with standard Euclidean alternatives
- generating smooth camera motion for rendering

### Requirements

Your submission should include:

1. **A ray-traced animation** generated from a moving camera following an interpolated trajectory.
2. **A trajectory definition** using at least several keyframe camera poses and Lie-group interpolation between them.
3. **A visualization of the camera trajectory**, for example by plotting camera centers and/or coordinate frames along the path.
4. **A comparison or discussion** of Lie-group interpolation versus a simpler Euclidean interpolation method, commenting on smoothness, consistency, or geometric correctness.
5. **A short report** (which may be included in a Jupyter notebook or submitted as a PDF) describing:
    - your trajectory construction method
    - the interpolation method used
    - implementation details
    - results and observations
6. **Code** in the form of Jupyter notebooks or Python scripts.

### Bonus (Optional)

Add the following experiments + theoretical descriptions: 

- (+20 pts): Manifold interpolation on Special Orthogonal Group versus full Special Euclidean Group
- (+20 pts): Perturbing the trajectory using tangent-space twists
- (+20 pts): Using left and right perturbations to study different motion behaviors
- (+20 pts): Generating more complex trajectories using multiple keyframes or closed loops

### Deliverables

Submit the URL of your GitHub repository containing:

- Source code
- Link to animation (video or rendered image sequence). Videos are usually large so just post a link to Vimeo, YouTube or similar hosting service.
- Report

The emphasis of this assignment is not only on producing a visually compelling animation, but also on using geometric methods to construct camera motion in a mathematically consistent way.