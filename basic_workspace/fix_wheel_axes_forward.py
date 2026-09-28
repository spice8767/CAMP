import xml.etree.ElementTree as ET

tree = ET.parse('/home/yash/buggy_demo_ws/src/CART_description/urdf/CART.xacro')
root = tree.getroot()

for joint in root.findall('.//joint'):
    if joint.get('type') == 'continuous':
        axis = joint.find('axis')
        if axis is not None:
            # Change from -1.0 to 1.0 to fix the inverted W/S
            axis.set('xyz', "1.0 0.0 0.0")

tree.write('/home/yash/buggy_demo_ws/src/CART_description/urdf/CART.xacro', xml_declaration=True)
