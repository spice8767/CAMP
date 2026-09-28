import xml.etree.ElementTree as ET

tree = ET.parse('/home/yash/buggy_demo_ws/src/CART_description/urdf/CART.xacro')
root = tree.getroot()

for origin in root.findall('.//origin'):
    xyz = origin.get('xyz')
    if xyz:
        # Scale all offsets by exactly 100
        coords = [float(x) * 100.0 for x in xyz.split()]
        origin.set('xyz', ' '.join([str(c) for c in coords]))

for mesh in root.findall('.//mesh'):
    # Original scale was 0.001. We are scaling the whole vehicle by 100.
    # 0.001 * 100 = 0.1
    mesh.set('scale', '0.1 0.1 0.1')

for link in root.findall('.//link'):
    inertial = link.find('inertial')
    if inertial is not None:
        mass_elem = inertial.find('mass')
        if mass_elem is not None:
            # Set a realistic mass for a golf cart (approx 400kg for body, 20kg for wheels)
            orig_mass = float(mass_elem.get('value'))
            if orig_mass > 0.001:
                mass = 400.0  # body
            else:
                mass = 20.0   # wheels or knuckles
            mass_elem.set('value', str(mass))
        else:
            mass = 1.0
            
        inertia = inertial.find('inertia')
        if inertia is not None:
            # Calculate a generic box inertia
            i_val = mass * 0.1
            if i_val < 0.01: i_val = 0.01
            inertia.set('ixx', str(i_val))
            inertia.set('iyy', str(i_val))
            inertia.set('izz', str(i_val))
            inertia.set('ixy', '0.0')
            inertia.set('iyz', '0.0')
            inertia.set('ixz', '0.0')

# Add Gazebo plugin
gazebo = ET.SubElement(root, 'gazebo')
plugin = ET.SubElement(gazebo, 'plugin', {'filename': 'gz-sim-ackermann-steering-system', 'name': 'gz::sim::systems::AckermannSteering'})

# RWD setup: Only drive the rear wheels to prevent physics fighting when turning
ET.SubElement(plugin, 'left_joint').text = 'Revolute 20'
ET.SubElement(plugin, 'right_joint').text = 'Revolute 21'

ET.SubElement(plugin, 'left_steering_joint').text = 'Revolute 22'
ET.SubElement(plugin, 'right_steering_joint').text = 'Revolute 23'

# The TRUE dimensions for the scaled-down buggy:
ET.SubElement(plugin, 'kingpin_width').text = '0.755'
ET.SubElement(plugin, 'steering_limit').text = '0.8'
ET.SubElement(plugin, 'wheel_base').text = '2.124'
ET.SubElement(plugin, 'wheel_radius').text = '0.277'

ET.SubElement(plugin, 'min_velocity').text = '-5'
ET.SubElement(plugin, 'max_velocity').text = '5'
ET.SubElement(plugin, 'min_acceleration').text = '-2'
ET.SubElement(plugin, 'max_acceleration').text = '2'
ET.SubElement(plugin, 'topic').text = '/cmd_vel'

tree.write('/home/yash/buggy_demo_ws/src/CART_description/urdf/CART.xacro', xml_declaration=True)
