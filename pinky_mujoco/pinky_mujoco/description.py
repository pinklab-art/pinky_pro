"""Expand the shared pinky_description xacro and resolve mesh URIs."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory


def expand_urdf(package='pinky_description', filename='urdf/robot.urdf.xacro', mappings=None):
    """Expand robot.urdf.xacro with is_sim:=false (is_sim only enables Gazebo plugins)."""
    import xacro
    options = {str(k): str(v) for k, v in (mappings or {}).items()}
    options['is_sim'] = 'false'
    path = Path(get_package_share_directory(package)) / filename
    return xacro.process_file(str(path), mappings=options).toxml()


def resolve_mesh(uri):
    if uri.startswith('package://'):
        package, relative = uri[len('package://'):].split('/', 1)
        return Path(get_package_share_directory(package)) / relative
    if uri.startswith('file://'):
        from urllib.parse import unquote, urlparse
        return Path(unquote(urlparse(uri).path))
    path = Path(uri)
    if not path.is_absolute():
        raise ValueError(f'Mesh must use package://, file://, or an absolute path: {uri}')
    return path
