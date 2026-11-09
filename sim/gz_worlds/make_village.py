"""Generate the village world: textured ground, two roads, houses, people and vehicles.

    python3 sim/gz_worlds/make_village.py [--seed 1] [--name village]

writes, next to this script,
    <name>.sdf          the Gazebo world (world name <name>; physics, light and spherical
                        coordinates as PX4's default world, so the vehicle and GPS behave the same)
    <name>_objects.csv  ground truth of every placed object, the reference for all
                        detection and localization metrics

Each seed gets its own name, so the worlds and their truth files sit side by side: village
(seed 1, the development flight) and village_seed2 (the test flight).

The layout is random but fully determined by the seed. Ground textures come from
setup/fetch_textures.sh; Fuel models are downloaded by Gazebo on the first start.

Truth convention (frames.csv of tools/export_bag.py uses the same): NED world, in metres.
An object's position is the centre of its footprint on the ground, not the model origin
(they differ by up to 0.34 m for the vehicles), because that is where the centre of a
nadir detection box projects to.
"""
import argparse
import csv
import math
import os
import random
from dataclasses import dataclass

HERE = os.path.dirname(os.path.abspath(__file__))
FUEL = 'https://fuel.gazebosim.org/1.0/OpenRobotics/models'


@dataclass(frozen=True)
class Mesh:
    """Mesh and texture of a Fuel model whose own material renders wrong in Gazebo Harmonic
    (OGRE material scripts draw black, broken texture paths draw white). The world then
    defines the model itself, with the texture as a PBR albedo map."""
    file: str            # path inside the Fuel model
    texture: str         # albedo map: a URI, or a file in textures/ (setup/fetch_textures.sh)
    scale: float
    yaw: float = 0.0     # visual yaw in the model frame (rad)


@dataclass(frozen=True)
class Model:
    uri: str
    cls: str
    height: float        # m
    length: float        # footprint along the model x axis (m)
    width: float         # footprint along the model y axis (m)
    center: tuple        # footprint centre in the model frame (m)
    mesh: Mesh | None = None  # set: build the model here instead of including it


def _hatchback(color):
    # The three Fuel textures share the file name hatchback.png, and Gazebo would draw all
    # cars with whichever loads first; fetch_textures.sh stores them under distinct names.
    return Mesh('meshes/hatchback.obj', f'textures/hatchback_{color}.png', 0.0254, math.pi / 2)


def _house(n):
    return Mesh(f'meshes/house_{n}.dae',
                f'{FUEL}/House%20{n}/3/files/materials/textures/House_{n}_Diffuse.png', 1.5)


# Measured from the meshes of the pinned Fuel versions (visual pose and scale applied).
PEOPLE = [
    Model(f'{FUEL}/Walking%20person/3', 'person', 1.87, 0.55, 0.88, (0.0, 0.035)),         # CC0
    Model(f'{FUEL}/Standing%20person/3', 'person', 1.92, 0.54, 0.36, (-0.005, -0.07)),     # CC0
    Model(f'{FUEL}/Casual%20female/4', 'person', 1.67, 0.61, 0.57, (0.015, -0.075)),       # CC0
    Model(f'{FUEL}/MaleVisitorOnPhone/2', 'person', 1.74, 0.46, 0.51, (0.12, -0.045)),     # CC-BY 4.0
]
VEHICLES = [
    Model(f'{FUEL}/Hatchback/3', 'vehicle', 1.57, 4.00, 2.14, (0.34, 0.0), _hatchback('white')),         # CC0
    Model(f'{FUEL}/Hatchback%20blue/5', 'vehicle', 1.57, 4.00, 2.14, (0.34, 0.0), _hatchback('blue')),   # CC0
    Model(f'{FUEL}/Hatchback%20red/4', 'vehicle', 1.57, 4.00, 2.14, (0.34, 0.0), _hatchback('red')),     # CC0
    Model(f'{FUEL}/Pickup/4', 'vehicle', 1.87, 5.66, 2.47, (-0.12, 0.0)),                  # CC0
    Model(f'{FUEL}/SUV/4', 'vehicle', 2.16, 5.02, 2.60, (0.07, 0.035)),                    # CC0
]
HOUSES = [
    Model(f'{FUEL}/House%201/3', 'building', 7.69, 16.49, 12.93, (-0.395, -0.375), _house(1)),  # CC0
    Model(f'{FUEL}/House%202/3', 'building', 7.23, 12.71, 9.77, (1.205, 0.775), _house(2)),     # CC0
    Model(f'{FUEL}/House%203/3', 'building', 9.17, 13.18, 8.71, (-0.49, -0.015), _house(3)),    # CC0
]

N_PEOPLE = 20
N_VEHICLES = 12
AREA = 28.0            # objects within |x|, |y| <= AREA (survey area is +-30 m)
HOME_CLEARANCE = 6.0   # m around the take-off point (origin) kept free
MIN_SPACING = 4.0      # m between people and vehicle centres: twice the 2 m match threshold
CLEARANCE = 1.0        # m between footprints
VEHICLE_CLEARANCE = 0.5  # m between vehicles: cars in the two lanes pass each other this close
HOUSE_CLEARANCE = 2.0  # m between a house and roads or other houses

TILE = 20.0            # ground tile size (m); texture 2048 px -> ~1 cm per texel
GROUND_HALF = 70.0     # textured ground covers |x|, |y| <= this
GRASS_SHARE = 0.3      # rest is Ground037, the texture with the most large-scale structure
ROAD_WIDTH = 7.0       # two 3.5 m lanes; roads are drawn as square tiles of this size
ROAD_OFFSET = 15.0     # road centre lines are drawn within +-this of the origin


@dataclass
class Placed:
    model: Model
    x: float           # footprint centre, ENU (m)
    y: float
    yaw: float         # ENU, rad

    def corners(self, margin=0.0):
        hl, hw = self.model.length / 2 + margin, self.model.width / 2 + margin
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return [(self.x + c * u - s * v, self.y + s * u + c * v)
                for u, v in ((hl, hw), (-hl, hw), (-hl, -hw), (hl, -hw))]

    def origin(self):
        """Model origin that puts the footprint centre at (x, y)."""
        cx, cy = self.model.center
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return self.x - (c * cx - s * cy), self.y - (s * cx + c * cy)


def overlaps(a, b):
    """Separating-axis test for two convex polygons (lists of corners)."""
    for poly in (a, b):
        for (x0, y0), (x1, y1) in zip(poly, poly[1:] + poly[:1]):
            nx, ny = y0 - y1, x1 - x0
            pa = [nx * x + ny * y for x, y in a]
            pb = [nx * x + ny * y for x, y in b]
            if max(pa) < min(pb) or max(pb) < min(pa):
                return False
    return True


def rect(cx, cy, length, width):
    hl, hw = length / 2, width / 2
    return [(cx + hl, cy + hw), (cx - hl, cy + hw), (cx - hl, cy - hw), (cx + hl, cy - hw)]


def layout(seed):
    rng = random.Random(seed)
    road_x = rng.uniform(-ROAD_OFFSET, ROAD_OFFSET)  # north-south road at x = road_x
    road_y = rng.uniform(-ROAD_OFFSET, ROAD_OFFSET)  # east-west road at y = road_y
    roads = [rect(road_x, 0.0, ROAD_WIDTH, 2 * GROUND_HALF), rect(0.0, road_y, 2 * GROUND_HALF, ROAD_WIDTH)]
    home = rect(0.0, 0.0, 2 * HOME_CLEARANCE, 2 * HOME_CLEARANCE)

    def sample(model, xy_yaw, keep_out, near, clearance, tries=10000):
        """Random pose whose footprint, grown by clearance, misses every keep-out polygon."""
        for _ in range(tries):
            p = Placed(model, *xy_yaw())
            poly = p.corners(clearance)
            if any(not (abs(x) <= AREA and abs(y) <= AREA) for x, y in p.corners()):
                continue
            if any(overlaps(poly, o) for o in keep_out):
                continue
            if any(math.hypot(p.x - q.x, p.y - q.y) < MIN_SPACING for q in near):
                continue
            return p
        raise RuntimeError(f'could not place {model.uri} (seed {seed}); relax the layout')

    def anywhere():
        return rng.uniform(-AREA, AREA), rng.uniform(-AREA, AREA), rng.choice([0, 1, 2, 3]) * math.pi / 2

    houses = []
    for model in HOUSES:
        keep_out = roads + [home] + [h.corners() for h in houses]
        houses.append(sample(model, anywhere, keep_out, [], HOUSE_CLEARANCE))

    def on_road():
        lane = rng.choice([-1, 1]) * ROAD_WIDTH / 4  # middle of either lane
        along = rng.uniform(-AREA, AREA)
        flip = rng.choice([0.0, math.pi])
        if rng.random() < 0.5:
            return road_x + lane, along, math.pi / 2 + flip
        return along, road_y + lane, flip

    crossing = rect(road_x, road_y, ROAD_WIDTH + 4, ROAD_WIDTH + 4)
    vehicles = []
    for i in range(N_VEHICLES):
        keep_out = [home, crossing] + [v.corners() for v in vehicles]
        vehicles.append(sample(VEHICLES[i % len(VEHICLES)], on_road, keep_out, vehicles, VEHICLE_CLEARANCE))

    def person_pose():
        return rng.uniform(-AREA, AREA), rng.uniform(-AREA, AREA), rng.uniform(-math.pi, math.pi)

    people = []
    for i in range(N_PEOPLE):
        keep_out = [home] + [h.corners() for h in houses] + [v.corners() for v in vehicles]
        people.append(sample(PEOPLE[i % len(PEOPLE)], person_pose, keep_out, people + vehicles, CLEARANCE))

    ground = [(x, y, rng.choice([0, 1, 2, 3]) * math.pi / 2,
               'Grass004' if rng.random() < GRASS_SHARE else 'Ground037')
              for x in _tile_centres(-GROUND_HALF, GROUND_HALF, TILE)
              for y in _tile_centres(-GROUND_HALF, GROUND_HALF, TILE)]
    # Road tiles start at the crossing, so the two roads meet without gaps or overlap.
    road_tiles = ([(road_x, y, rng.choice([0, 1, 2, 3]) * math.pi / 2)
                   for y in _tiles_through(road_y, GROUND_HALF, ROAD_WIDTH)]
                  + [(x, road_y, rng.choice([0, 1, 2, 3]) * math.pi / 2)
                     for x in _tiles_through(road_x, GROUND_HALF, ROAD_WIDTH) if x != road_x])
    return houses, vehicles, people, ground, road_tiles


def _tile_centres(lo, hi, size):
    n = math.ceil((hi - lo) / size)
    return [lo + size * (i + 0.5) for i in range(n)]


def _tiles_through(c, half, size):
    """Centres of size-wide tiles, one centred on c, covering [-half, half]."""
    k_lo, k_hi = math.floor((-half - c) / size + 0.5), math.ceil((half - c) / size - 0.5)
    return [c + k * size for k in range(k_lo, k_hi + 1)]


def _tile(name, x, y, z, yaw, size, texture):
    return f"""      <visual name="{name}">
        <pose>{x:.3f} {y:.3f} {z} 0 0 {yaw:.4f}</pose>
        <geometry><plane><normal>0 0 1</normal><size>{size} {size}</size></plane></geometry>
        <material>
          <ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse><specular>0 0 0 1</specular>
          <pbr><metal><albedo_map>textures/{texture}.jpg</albedo_map><roughness>1</roughness><metalness>0</metalness></metal></pbr>
        </material>
      </visual>
"""


def _include(name, p):
    ox, oy = p.origin()
    if p.model.mesh is not None:
        return _own_model(name, p, ox, oy)
    return f"""    <include>
      <uri>{p.model.uri}</uri>
      <name>{name}</name>
      <pose>{ox:.3f} {oy:.3f} 0 0 0 {p.yaw:.4f}</pose>
      <static>true</static>
    </include>
"""


def _own_model(name, p, ox, oy):
    m = p.model.mesh
    geometry = (f'<geometry><mesh><uri>{p.model.uri}/files/{m.file}</uri>'
                f'<scale>{m.scale} {m.scale} {m.scale}</scale></mesh></geometry>')
    return f"""    <model name="{name}">
      <static>true</static>
      <pose>{ox:.3f} {oy:.3f} 0 0 0 {p.yaw:.4f}</pose>
      <link name="link">
        <collision name="collision">
          <pose>0 0 0 0 0 {m.yaw:.4f}</pose>
          {geometry}
        </collision>
        <visual name="visual">
          <pose>0 0 0 0 0 {m.yaw:.4f}</pose>
          {geometry}
          <material>
            <diffuse>1 1 1 1</diffuse><specular>0.2 0.2 0.2 1</specular>
            <pbr><metal><albedo_map>{m.texture}</albedo_map><roughness>0.8</roughness><metalness>0</metalness></metal></pbr>
          </material>
        </visual>
      </link>
    </model>
"""


def world_sdf(seed, name, houses, vehicles, people, ground, road_tiles):
    tiles = ''.join(_tile(f'ground_{i}', x, y, 0, yaw, TILE, tex) for i, (x, y, yaw, tex) in enumerate(ground))
    tiles += ''.join(_tile(f'road_{i}', x, y, 0.01, yaw, ROAD_WIDTH, 'Asphalt031')
                     for i, (x, y, yaw) in enumerate(road_tiles))
    objects = ''.join(_include(name, p) for name, p in named(houses, vehicles, people))
    # Physics, scene, light and spherical coordinates as PX4's Tools/simulation/gz/worlds/default.sdf.
    return f"""<?xml version="1.0" encoding="UTF-8"?>
<!-- Generated by sim/gz_worlds/make_village.py with seed {seed}. Do not edit by hand. -->
<sdf version="1.9">
  <world name="{name}">
    <physics type="ode">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>250</real_time_update_rate>
    </physics>
    <gravity>0 0 -9.8</gravity>
    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    <atmosphere type="adiabatic"/>
    <scene>
      <grid>false</grid>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>true</shadows>
    </scene>
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>500 500</size></plane></geometry>
        </collision>
        <visual name="far_ground">
          <pose>0 0 -0.02 0 0 0</pose>
          <geometry><plane><normal>0 0 1</normal><size>500 500</size></plane></geometry>
          <material>
            <ambient>0.5 0.5 0.45 1</ambient><diffuse>0.5 0.5 0.45 1</diffuse><specular>0 0 0 1</specular>
          </material>
        </visual>
{tiles}      </link>
    </model>
    <light name="sunUTC" type="directional">
      <pose>0 0 500 0 -0 0</pose>
      <cast_shadows>true</cast_shadows>
      <intensity>1</intensity>
      <direction>0.001 0.625 -0.78</direction>
      <diffuse>0.904 0.904 0.904 1</diffuse>
      <specular>0.271 0.271 0.271 1</specular>
      <attenuation>
        <range>2000</range>
        <linear>0</linear>
        <constant>1</constant>
        <quadratic>0</quadratic>
      </attenuation>
    </light>
{objects}    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>47.397971057728974</latitude_deg>
      <longitude_deg>8.546163739800146</longitude_deg>
      <elevation>0</elevation>
    </spherical_coordinates>
  </world>
</sdf>
"""


def named(houses, vehicles, people):
    for prefix, group in (('house', houses), ('vehicle', vehicles), ('person', people)):
        for i, p in enumerate(group):
            yield f'{prefix}_{i:02d}', p


def write_truth(path, houses, vehicles, people):
    with open(path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['name', 'class', 'model', 'n', 'e', 'd', 'yaw_deg', 'height', 'length', 'width'])
        for name, p in named(houses, vehicles, people):
            yaw_ned = math.degrees(math.atan2(math.cos(p.yaw), math.sin(p.yaw)))  # pi/2 - yaw_enu
            w.writerow([name, p.model.cls, p.model.uri.rsplit('/', 2)[1].replace('%20', ' '),
                        f'{p.y:.3f}', f'{p.x:.3f}', '0.000', f'{yaw_ned:.1f}',
                        p.model.height, p.model.length, p.model.width])


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--seed', type=int, default=1)
    parser.add_argument('--name', default='village', help='world name and file stem')
    args = parser.parse_args()
    houses, vehicles, people, ground, road_tiles = layout(args.seed)
    with open(os.path.join(HERE, f'{args.name}.sdf'), 'w') as f:
        f.write(world_sdf(args.seed, args.name, houses, vehicles, people, ground, road_tiles))
    write_truth(os.path.join(HERE, f'{args.name}_objects.csv'), houses, vehicles, people)
    print(f'seed {args.seed}: {len(houses)} houses, {len(vehicles)} vehicles, {len(people)} people, '
          f'{len(ground)} ground tiles, {len(road_tiles)} road tiles')


if __name__ == '__main__':
    main()
