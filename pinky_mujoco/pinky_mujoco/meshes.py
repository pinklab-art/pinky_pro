"""Convert visual meshes (DAE/OBJ/STL) into per-material MuJoCo assets, keeping colours and textures.

MuJoCo loads STL/OBJ/MSH only and one material per mesh, so a DAE with several materials is
split into one OBJ per sub-mesh. Texture images are written as PNG next to the OBJ files.
Results are cached under output_dir and rebuilt when the source file is newer.
"""
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np


def _rgba(color):
    values = np.asarray(color, dtype=float).ravel()
    if values.size == 3:
        values = np.append(values, 255.0)
    if values.max() > 1.0:
        values = values / 255.0
    return [float(x) for x in values[:4]]


def _material_info(mesh):
    """(texture PIL image or None, rgba list) from a trimesh visual."""
    visual = mesh.visual
    image, color = None, [0.8, 0.8, 0.8, 1.0]
    if getattr(visual, 'kind', None) == 'texture':
        material = visual.material
        image = getattr(material, 'image', None)
        if image is None:
            image = getattr(material, 'baseColorTexture', None)
        diffuse = getattr(material, 'diffuse', None)
        if diffuse is None:
            diffuse = getattr(material, 'baseColorFactor', None)
        if diffuse is not None:
            color = _rgba(diffuse)
        if image is not None and (visual.uv is None or len(visual.uv) != len(mesh.vertices)):
            image = None  # texture without usable UVs: fall back to the diffuse colour
    elif getattr(visual, 'kind', None) == 'face' or getattr(visual, 'kind', None) == 'vertex':
        color = _rgba(visual.main_color)
    return image, color


def write_obj(path, vertices, faces, uv=None):
    lines = [f'v {x:.7g} {y:.7g} {z:.7g}' for x, y, z in vertices]
    if uv is not None:
        lines += [f'vt {u:.6g} {v:.6g}' for u, v in uv]
        lines += [f'f {a + 1}/{a + 1} {b + 1}/{b + 1} {c + 1}/{c + 1}' for a, b, c in faces]
    else:
        lines += [f'f {a + 1} {b + 1} {c + 1}' for a, b, c in faces]
    Path(path).write_text('\n'.join(lines) + '\n')


def convert_visual_mesh(source, output_dir):
    """Return a list of parts: {'file': obj/stl path, 'texture': png path or None, 'rgba': [r,g,b,a]}."""
    source, output_dir = Path(source), Path(output_dir)
    if source.suffix.lower() in ('.stl', '.obj', '.msh'):
        return [{'file': str(source.resolve()), 'texture': None, 'rgba': None}]
    digest = hashlib.sha256(str(source.resolve()).encode()).hexdigest()[:10]
    folder = output_dir / f'{source.stem}_{digest}'
    manifest = folder / 'parts.json'
    if manifest.exists() and manifest.stat().st_mtime >= source.stat().st_mtime:
        parts = json.loads(manifest.read_text())
        if all(Path(p['file']).exists() and (p['texture'] is None or Path(p['texture']).exists()) for p in parts):
            return parts
    import trimesh
    folder.mkdir(parents=True, exist_ok=True)
    scene = trimesh.load(str(source), force='scene')
    parts = []
    for index, mesh in enumerate(scene.dump(concatenate=False)):
        if not isinstance(mesh, trimesh.Trimesh) or len(mesh.faces) == 0:
            continue
        image, rgba = _material_info(mesh)
        texture = None
        uv = None
        if image is not None:
            texture = folder / f'part{index}.png'
            image.convert('RGBA').save(texture)
            uv = np.asarray(mesh.visual.uv, dtype=float)
        obj = folder / f'part{index}.obj'
        write_obj(obj, mesh.vertices, mesh.faces, uv)
        parts.append({'file': str(obj.resolve()), 'texture': str(texture.resolve()) if texture else None,
                      'rgba': rgba})
    manifest.write_text(json.dumps(parts, indent=1))
    return parts


def textured_quad(output_dir, name, size_xy, texture_source):
    """A flat textured rectangle (for SDF plane visuals with an albedo map)."""
    from PIL import Image
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    half = np.asarray(size_xy, dtype=float) / 2
    obj = output_dir / f'{name}.obj'
    png = output_dir / f'{name}.png'
    if not png.exists() or png.stat().st_mtime < Path(texture_source).stat().st_mtime:
        Image.open(texture_source).convert('RGB').save(png)
    vertices = np.array([[-half[0], -half[1], 0], [half[0], -half[1], 0], [half[0], half[1], 0], [-half[0], half[1], 0]])
    # Image row 0 is the top edge, which lies at +Y (the far edge when looking along +X from the floor origin).
    uv = np.array([[0, 0], [1, 0], [1, 1], [0, 1]], dtype=float)
    write_obj(obj, vertices, [[0, 1, 2], [0, 2, 3]], uv)
    return [{'file': str(obj.resolve()), 'texture': str(png.resolve()), 'rgba': [1, 1, 1, 1]}]


class AssetRegistry:
    """Adds <mesh>/<texture>/<material> elements once per (file, scale) and hands back geom attributes."""
    def __init__(self, asset_element, prefix=''):
        self.asset = asset_element
        self.prefix = prefix  # keeps robot and world asset names apart in one MJCF
        self.meshes, self.materials = {}, {}

    def geom_attrs(self, part, scale='1 1 1', default_rgba='0.8 0.8 0.82 1'):
        key = (part['file'], scale)
        if key not in self.meshes:
            name = f'{self.prefix}mesh_{len(self.meshes)}'
            # Inertia never comes from these meshes (bodies carry URDF inertials, world geoms are
            # static); 'shell' also accepts zero-volume parts such as textured quads.
            ET.SubElement(self.asset, 'mesh', name=name, file=part['file'], scale=scale, inertia='shell')
            self.meshes[key] = name
        attrs = {'type': 'mesh', 'mesh': self.meshes[key]}
        if part.get('texture'):
            if part['texture'] not in self.materials:
                tex, mat = f'{self.prefix}tex_{len(self.materials)}', f'{self.prefix}mat_{len(self.materials)}'
                ET.SubElement(self.asset, 'texture', name=tex, type='2d', file=part['texture'])
                ET.SubElement(self.asset, 'material', name=mat, texture=tex, specular='0.1', shininess='0.1')
                self.materials[part['texture']] = mat
            attrs['material'] = self.materials[part['texture']]
        elif part.get('rgba'):
            attrs['rgba'] = ' '.join(f'{x:.4g}' for x in part['rgba'])
        else:
            attrs['rgba'] = default_rgba
        return attrs
