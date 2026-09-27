import xml.etree.ElementTree as ET

tree = ET.parse('/home/yash/buggy_demo_ws/src/CART_description/urdf/CART.xacro')
root = tree.getroot()

for joint in root.findall('.//joint'):
    if joint.get('type') == 'continuous':
        axis = joint.find('axis')
        if axis is not None:
            xyz = axis.get('xyz')
            if xyz:
                # Invert the X axis so wheels spin forward instead of backward
                x, y, z = map(float, xyz.split())
                axis.set('xyz', f"{-x} {y} {z}")

tree.write('/home/yash/buggy_demo_ws/src/CART_description/urdf/CART.xacro', xml_declaration=True)
