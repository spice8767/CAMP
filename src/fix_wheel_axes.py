import xml.etree.ElementTree as ET

tree = ET.parse('/home/yash/buggy_demo_ws/src/CART_description/urdf/CART.xacro')
root = tree.getroot()

for joint in root.findall('.//joint'):
    if joint.get('type') == 'continuous':
        axis = joint.find('axis')
        if axis is not None:
            # Force all wheels to have exactly the same axis direction
            # -1.0 0.0 0.0 pushes the buggy forward when positive velocity is applied
            axis.set('xyz', "-1.0 0.0 0.0")

tree.write('/home/yash/buggy_demo_ws/src/CART_description/urdf/CART.xacro', xml_declaration=True)
