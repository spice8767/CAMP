import re

file_path = '/home/adarsh4our/CAMP A/src/saye_description/models/saye/model.xacro'

with open(file_path, 'r') as f:
    content = f.read()

# Links to update to 400.0 / 40.0
links_400 = ['base_link', 'top_1']
# Links to update to 20.0 / 2.0
links_20 = [
    'wheel_front_left_1', 'wheel_front_right_1', 
    'wheel_back_left_1', 'wheel_back_right_1',
    'Knuckle_left_1', 'knuckle_right_1'
]

def replace_mass_inertia(match, new_mass, new_inertia):
    mass_str = f'<mass value="{new_mass:.1f}" />'
    inertia_str = f'<inertia ixx="{new_inertia:.1f}" ixy="0.0" ixz="0.0" iyy="{new_inertia:.1f}" iyz="0.0" izz="{new_inertia:.1f}" />'
    
    # We replace whatever the mass and inertia currently are
    part = match.group(0)
    part = re.sub(r'<mass\s+value="[^"]+"\s*/>', mass_str, part)
    part = re.sub(r'<inertia\s+ixx="[^"]+"\s+ixy="[^"]+"\s+ixz="[^"]+"\s+iyy="[^"]+"\s+iyz="[^"]+"\s+izz="[^"]+"\s*/>', inertia_str, part)
    return part

for link in links_400:
    pattern = rf'(<link\s+name="{link}">.*?<inertial>.*?</inertial>)'
    content = re.sub(pattern, lambda m: replace_mass_inertia(m, 400.0, 40.0), content, flags=re.DOTALL)

for link in links_20:
    pattern = rf'(<link\s+name="{link}">.*?<inertial>.*?</inertial>)'
    content = re.sub(pattern, lambda m: replace_mass_inertia(m, 20.0, 2.0), content, flags=re.DOTALL)

with open(file_path, 'w') as f:
    f.write(content)
