import xml.etree.ElementTree as ET

tree = ET.parse('/home/yash/buggy_demo_ws/src/CART_description/urdf/CART.xacro')
root = tree.getroot()

for link in root.findall('.//link'):
    inertial = link.find('inertial')
    if inertial is not None:
        mass_elem = inertial.find('mass')
        if mass_elem is not None:
            mass = float(mass_elem.get('value'))
        else:
            mass = 1.0
            
        inertia = inertial.find('inertia')
        if inertia is not None:
            # Sphere inertia: 2/5 * m * r^2. Let's assume radius = 0.3m -> 2/5 * m * 0.09 = m * 0.036
            # We'll just use a generic I = mass * 0.1 for diagonal, 0 for cross
            i_val = mass * 0.1
            if i_val < 0.01:
                i_val = 0.01 # prevent 0 inertia
            inertia.set('ixx', str(i_val))
            inertia.set('iyy', str(i_val))
            inertia.set('izz', str(i_val))
            inertia.set('ixy', '0.0')
            inertia.set('iyz', '0.0')
            inertia.set('ixz', '0.0')

tree.write('/home/yash/buggy_demo_ws/src/CART_description/urdf/CART.xacro', xml_declaration=True)
