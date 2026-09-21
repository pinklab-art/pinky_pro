"""Load a Gazebo SDF world (pinky_gz_sim/worlds/*.world) or a simple box list (worlds/*.yaml) into MuJoCo.

SDF support covers what the Pinky worlds use: nested <model>/<link> poses, <include> of
model:// directories, box / cylinder / sphere / mesh geometry. Planes (ground, logo) are
skipped because the MuJoCo scene already has a floor. Lights, plugins and sensors are ignored.
"""
import os
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import yaml
from scipy.spatial.transform import Rotation as R
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from .meshes import convert_visual_mesh, textured_quad, AssetRegistry


def vector(values):
    return ' '.join(f'{float(x):.12g}' for x in values)


def numbers(text, default):
    return np.array([float(x) for x in text.split()]) if text and text.strip() else np.array(default, dtype=float)


def pose_matrix(element):
    """SDF <pose>x y z roll pitch yaw</pose> (fixed-axis rpy) as a 4x4 matrix."""
    tag = element.find('pose') if element is not None else None
    values = numbers(tag.text if tag is not None else None, [0.0] * 6)
    matrix = np.eye(4)
    matrix[:3, :3] = R.from_euler('xyz', values[3:]).as_matrix()
    matrix[:3, 3] = values[:3]
    return matrix


def pose_attrs(matrix):
    x, y, z, w = R.from_matrix(matrix[:3, :3]).as_quat()
    return {'pos': vector(matrix[:3, 3]), 'quat': vector([w, x, y, z])}


def model_search_paths():
    paths = []
    for variable in ('GZ_SIM_RESOURCE_PATH', 'IGN_GAZEBO_RESOURCE_PATH', 'GAZEBO_MODEL_PATH'):
        paths += [Path(p) for p in os.environ.get(variable, '').split(':') if p]
    try:
        paths.append(Path(get_package_share_directory('pinky_gz_sim')) / 'models')
    except PackageNotFoundError:
        pass
    paths.append(Path.home() / '.gazebo/models')
    return paths


def resolve_uri(uri, base_dir):
    uri = uri.strip()
    if uri.startswith('model://'):
        relative = uri[len('model://'):]
        for root in model_search_paths():
            candidate = root / relative
            if candidate.exists():
                return candidate
        raise FileNotFoundError(f'Gazebo model not found on GZ_SIM_RESOURCE_PATH or pinky_gz_sim/models: {uri}')
    if uri.startswith('file://'):
        path = Path(uri[len('file://'):])
        if path.is_absolute():
            return path
        # Gazebo resolves relative file:// URIs on GZ_SIM_RESOURCE_PATH (e.g. models/img/pinklab.jpg).
        for root in [base_dir, *model_search_paths()]:
            if (Path(root) / path).exists():
                return Path(root) / path
        raise FileNotFoundError(f'Resource not found: {uri}')
    if uri.startswith('package://'):
        package, relative = uri[len('package://'):].split('/', 1)
        return Path(get_package_share_directory(package)) / relative
    return Path(base_dir) / uri


def model_sdf_path(model_dir):
    config = model_dir / 'model.config'
    if config.exists():
        sdf = ET.parse(config).getroot().find('sdf')
        if sdf is not None and sdf.text:
            return model_dir / sdf.text.strip()
    return model_dir / 'model.sdf'


class WorldBuilder:
    def __init__(self, worldbody, asset, output_dir):
        self.worldbody, self.asset, self.output_dir = worldbody, asset, Path(output_dir)
        self.registry = AssetRegistry(asset, prefix='world_')
        self.count = 0

    # ---- entry points -----------------------------------------------------
    def load(self, path):
        path = Path(path)
        if path.suffix == '.yaml':
            self.load_yaml(path)
        else:
            self.load_sdf(path)

    def load_yaml(self, path):
        boxes = yaml.safe_load(path.read_text())['boxes']
        for i, box in enumerate(boxes):
            ET.SubElement(self.worldbody, 'geom', name=box['name'], type='box',
                          pos=vector(box['center']), size=vector(np.array(box['size']) / 2),
                          group='0', contype='1', conaffinity='2',
                          rgba='0.35 0.46 0.55 1' if i < 4 else '0.75 0.48 0.27 1')

    def load_sdf(self, path):
        root = ET.parse(path).getroot()
        world = root.find('world') if root.tag == 'sdf' else root
        if world is None:
            raise ValueError(f'No <world> in {path}')
        self.add_children(world, np.eye(4), path.parent, '')

    # ---- SDF tree ---------------------------------------------------------
    def add_children(self, element, parent, base_dir, prefix):
        for model in element.findall('model'):
            self.add_model(model, parent, base_dir, prefix)
        for include in element.findall('include'):
            self.add_include(include, parent, base_dir, prefix)

    def add_include(self, include, parent, base_dir, prefix):
        model_dir = resolve_uri(include.findtext('uri'), base_dir)
        sdf = ET.parse(model_sdf_path(model_dir)).getroot()
        model = sdf.find('model')
        if model is None:
            raise ValueError(f'No <model> in {model_dir}')
        name = include.findtext('name') or model.get('name', 'model')
        # The include pose replaces the model's own pose.
        world_from_model = parent @ (pose_matrix(include) if include.find('pose') is not None
                                     else pose_matrix(model))
        self.add_model_body(model, world_from_model, model_dir, f'{prefix}{name}')

    def add_model(self, model, parent, base_dir, prefix):
        self.add_model_body(model, parent @ pose_matrix(model), base_dir,
                            f"{prefix}{model.get('name', 'model')}")

    def add_model_body(self, model, world_from_model, base_dir, name):
        for link in model.findall('link'):
            world_from_link = world_from_model @ pose_matrix(link)
            link_name = f"{name}/{link.get('name', 'link')}"
            has_visual = any(self.geometry_kind(v) is not None for v in link.findall('visual'))
            for collision in link.findall('collision'):
                self.add_geom(collision, world_from_link, base_dir, link_name,
                              visual=False, hidden=has_visual)
            for visual in link.findall('visual'):
                self.add_geom(visual, world_from_link, base_dir, link_name, visual=True, hidden=False)
        self.add_children(model, world_from_model, base_dir, f'{name}/')

    # ---- geometry ---------------------------------------------------------
    @staticmethod
    def geometry_kind(element):
        geometry = element.find('geometry')
        if geometry is None or len(geometry) == 0:
            return None
        return geometry[0].tag

    def add_geom(self, element, world_from_link, base_dir, link_name, visual, hidden):
        kind = self.geometry_kind(element)
        if kind in (None, 'heightmap') or (kind == 'plane' and not visual):
            return
        shape = element.find('geometry')[0]
        self.count += 1
        pose = world_from_link @ pose_matrix(element)
        if kind == 'plane':
            self.add_plane_visual(element, shape, pose, link_name)
            return
        attrs = pose_attrs(pose)
        attrs['name'] = f"{link_name}/{element.get('name', kind)}_{self.count}"
        if visual:
            attrs.update(group='2', contype='0', conaffinity='0', rgba=self.color(element, '0.8 0.8 0.82 1'))
        else:
            # Lidar rays test groups 0 and 4 (robot collisions are group 1, visuals 2).
            # MuJoCo rays skip alpha-0 geoms, so collisions duplicated by a visual go to
            # group 4: opaque for the ray cast, hidden by the viewer's default group mask.
            attrs.update(group='4' if hidden else '0', contype='1', conaffinity='2',
                         rgba='0.4 0.4 0.4 1' if hidden else self.color(element, '0.55 0.57 0.6 1'))
        if kind == 'box':
            attrs.update(type='box', size=vector(numbers(shape.findtext('size'), [1, 1, 1]) / 2))
        elif kind == 'cylinder':
            attrs.update(type='cylinder', size=vector([float(shape.findtext('radius')),
                                                       float(shape.findtext('length')) / 2]))
        elif kind == 'sphere':
            attrs.update(type='sphere', size=shape.findtext('radius').strip())
        elif kind == 'mesh':
            path = resolve_uri(shape.findtext('uri'), base_dir)
            scale = vector(numbers(shape.findtext('scale'), [1, 1, 1]))
            if visual:
                base = attrs['name']
                for i, part in enumerate(convert_visual_mesh(path, self.output_dir)):
                    part_attrs = {**attrs, 'name': f'{base}_{i}'}
                    part_attrs.pop('rgba', None)
                    part_attrs.update(self.registry.geom_attrs(part, scale))
                    ET.SubElement(self.worldbody, 'geom', **part_attrs)
                return
            attrs.update(self.registry.geom_attrs({'file': str(path.resolve())}, scale))
            attrs['rgba'] = '0.4 0.4 0.4 1' if hidden else self.color(element, '0.55 0.57 0.6 1')
        else:
            raise ValueError(f'Unsupported SDF geometry <{kind}> in {link_name}')
        ET.SubElement(self.worldbody, 'geom', **attrs)

    def add_plane_visual(self, element, shape, pose, link_name):
        """Gazebo ground planes are skipped; a plane with an albedo texture becomes a flat textured quad."""
        albedo = element.find('material/pbr/metal/albedo_map')
        if albedo is None or not albedo.text:
            return
        image = resolve_uri(albedo.text, self.output_dir)
        size = numbers(shape.findtext('size'), [1, 1])
        name = f"{link_name}/{element.get('name', 'plane')}_{self.count}".replace('/', '_')
        pose = pose.copy()
        pose[2, 3] += 0.002  # lift above the MuJoCo floor to avoid z-fighting
        attrs = pose_attrs(pose)
        attrs.update(name=name, group='2', contype='0', conaffinity='0')
        attrs.update(self.registry.geom_attrs(textured_quad(self.output_dir, name, size, image)[0]))
        ET.SubElement(self.worldbody, 'geom', **attrs)

    @staticmethod
    def color(element, default):
        diffuse = element.find('material/diffuse')
        if diffuse is None or not diffuse.text:
            return default
        values = numbers(diffuse.text, [0.8, 0.8, 0.8, 1])
        return vector(values if len(values) == 4 else [*values[:3], 1.0])
