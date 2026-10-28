# Village world credits

The repository contains no third-party assets: `village.sdf` refers to the models by
their Gazebo Fuel URIs (Gazebo downloads them on the first start) and
`setup/fetch_textures.sh` downloads the textures.

## Textures

| Asset | Source | License |
|---|---|---|
| Grass004, Ground037, Asphalt031 (2K color maps) | [ambientCG](https://ambientcg.com) | CC0 1.0 |
| Hatchback textures (white, blue, red) | Gazebo Fuel, OpenRobotics: Hatchback, Hatchback blue, Hatchback red | CC0 1.0 |

## Models (Gazebo Fuel, owner OpenRobotics)

| Model | Version | License |
|---|---|---|
| Walking person | 3 | CC0 1.0 |
| Standing person | 3 | CC0 1.0 |
| Casual female | 4 | CC0 1.0 |
| MaleVisitorOnPhone | 2 | **CC BY 4.0**: "MaleVisitorOnPhone" by OpenRobotics, https://fuel.gazebosim.org/1.0/OpenRobotics/models/MaleVisitorOnPhone, licensed under [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/); used unmodified |
| Hatchback, Hatchback blue, Hatchback red | 3, 5, 4 | CC0 1.0 |
| Pickup | 4 | CC0 1.0 |
| SUV | 4 | CC0 1.0 |
| House 1, House 2, House 3 | 3 | CC0 1.0 |

The hatchbacks and houses are drawn with their own Fuel meshes and textures; only the way the
texture is attached differs (PBR albedo map instead of the original material), see
`make_village.py`.

## World settings

Physics, scene, sun and spherical coordinates are copied from PX4's
`Tools/simulation/gz/worlds/default.sdf` (PX4-Autopilot, BSD 3-Clause).
