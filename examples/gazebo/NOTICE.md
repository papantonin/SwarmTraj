# Upstream model attribution

These test fixtures use [PX4 Gazebo Models](https://github.com/PX4/PX4-gazebo-models),
Copyright (c) 2022, PX4 Autopilot for Drones, under the BSD 3-Clause license
reproduced in [LICENSE.PX4](LICENSE.PX4).

- `models/rc_cessna_lidar/model.sdf` inherits the pinned image's `rc_cessna`
  model and adapts the downward single-beam lidar convention from
  `models/x500_lidar_down/model.sdf`. The extra sensor link is an illustrative
  test fixture, not a calibrated accessory.
- `worlds/default.sdf` adapts the image's `default.sdf` ground dimensions while
  retaining its physics, atmosphere, magnetic field and geographic origin.
- `meshes/ground.dae` and `materials/textures/grass.ppm` are original lightweight
  visual fixtures: two flat triangles and a deterministic 32-by-32 grass/field
  motif. They are not copied photographic imagery or georeferenced terrain.
- The enabled static skybox is supplied by Gazebo Rendering in the runtime
  image; it is not copied into this repository.
- The original aircraft assets and aerodynamic plugins are supplied by the
  pinned official runtime image, not copied or replaced in this directory.

These adaptations are not endorsed or certified by PX4 or Gazebo contributors.
