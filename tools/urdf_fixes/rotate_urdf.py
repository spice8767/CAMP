import xml.etree.ElementTree as ET
import math

tree = ET.parse('/home/yash/buggy_demo_ws/src/CART_description/urdf/CART.xacro')
root = tree.getroot()

def transform_xyz(xyz_str):
    if not xyz_str: return "0 0 0"
    x, y, z = map(float, xyz_str.split())
    # X_new = -Y_old
    # Y_new = X_old
    # Z_new = Z_old
    return f"{-y} {x} {z}"

def transform_rpy(rpy_str):
    if not rpy_str: return "0 0 0"
    r, p, y = map(float, rpy_str.split())
    # Add 90 degrees to Yaw to match the physical rotation of the vectors
    return f"{r} {p} {y + 1.57079632679}"

for origin in root.findall('.//origin'):
    xyz = origin.get('xyz')
    if xyz:
        origin.set('xyz', transform_xyz(xyz))
    
    rpy = origin.get('rpy')
    if rpy:
        origin.set('rpy', transform_rpy(rpy))
    else:
        origin.set('rpy', '0 0 1.57079632679')

for axis in root.findall('.//axis'):
    xyz = axis.get('xyz')
    if xyz:
        axis.set('xyz', transform_xyz(xyz))

for inertia in root.findall('.//inertia'):
    ixx = float(inertia.get('ixx', '0'))
    iyy = float(inertia.get('iyy', '0'))
    # Swap Ixx and Iyy
    inertia.set('ixx', str(iyy))
    inertia.set('iyy', str(ixx))
    
    # Ixy needs to be negated if we rotated by 90 deg, but it's 0.0 anyway.
    ixy = float(inertia.get('ixy', '0'))
    inertia.set('ixy', str(-ixy))

tree.write('/home/yash/buggy_demo_ws/src/CART_description/urdf/CART.xacro', xml_declaration=True)
