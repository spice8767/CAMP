import xml.etree.ElementTree as ET

tree = ET.parse('/home/yash/buggy_demo_ws/src/CART_description/urdf/CART.xacro')
root = tree.getroot()

for origin in root.findall('.//origin'):
    xyz = origin.get('xyz')
    if xyz:
        coords = [float(x) * 1000.0 for x in xyz.split()]
        origin.set('xyz', ' '.join([str(c) for c in coords]))

for mesh in root.findall('.//mesh'):
    mesh.set('scale', '1.0 1.0 1.0')

for link in root.findall('.//link'):
    inertial = link.find('inertial')
    if inertial is not None:
        mass_elem = inertial.find('mass')
        if mass_elem is not None:
            # Original mass was super tiny, e.g. 0.0014. Multiply by 1,000,000 -> 1400kg
            mass = float(mass_elem.get('value')) * 1000000.0
            mass_elem.set('value', str(mass))
        else:
            mass = 1.0
            
        inertia = inertial.find('inertia')
        if inertia is not None:
            i_val = mass * 0.1
            if i_val < 0.01:
                i_val = 0.01 # prevent 0 inertia
            inertia.set('ixx', str(i_val))
            inertia.set('iyy', str(i_val))
            inertia.set('izz', str(i_val))
            inertia.set('ixy', '0.0')
            inertia.set('iyz', '0.0')
            inertia.set('ixz', '0.0')

# Add Gazebo plugin
gazebo = ET.SubElement(root, 'gazebo')
plugin = ET.SubElement(gazebo, 'plugin', {'filename': 'gz-sim-ackermann-steering-system', 'name': 'gz::sim::systems::AckermannSteering'})

ET.SubElement(plugin, 'left_joint').text = 'Revolute 10'
ET.SubElement(plugin, 'right_joint').text = 'Revolute 11'
ET.SubElement(plugin, 'left_joint').text = 'Revolute 20'
ET.SubElement(plugin, 'right_joint').text = 'Revolute 21'
ET.SubElement(plugin, 'left_steering_joint').text = 'Revolute 22'
ET.SubElement(plugin, 'right_steering_joint').text = 'Revolute 23'

# The TRUE dimensions based on the joint origins in the URDF:
ET.SubElement(plugin, 'kingpin_width').text = '7.55'
ET.SubElement(plugin, 'steering_limit').text = '0.6'
ET.SubElement(plugin, 'wheel_base').text = '21.24'
ET.SubElement(plugin, 'wheel_radius').text = '2.77'

ET.SubElement(plugin, 'min_velocity').text = '-10'
ET.SubElement(plugin, 'max_velocity').text = '10'
ET.SubElement(plugin, 'min_acceleration').text = '-3'
ET.SubElement(plugin, 'max_acceleration').text = '3'
ET.SubElement(plugin, 'topic').text = '/cmd_vel'

tree.write('/home/yash/buggy_demo_ws/src/CART_description/urdf/CART.xacro', xml_declaration=True)
