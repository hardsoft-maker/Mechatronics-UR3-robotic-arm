from lxml import etree

# Load the XML model and build the XML tree
tree = etree.parse('mjmodel.xml')
root = tree.getroot()

# Define the reflected inertia value to be added to the joints
reflected_inertia = 0.01

# Find all joints inside the XML model
joints = root.xpath('//joint')

# Add the reflected inertia to the joints via the armature parameter
for joint in joints:
    joint.set('armature', str(reflected_inertia))

# Change mass and inertia diagonal due to motor changes
# Assuming we are updating the mass and inertia diagonal for specific bodies
bodies_to_update = {
    'shoulder_link': {'mass': 2.5, 'diaginertia': '0.009 0.009 0.006'},
    'upper_arm_link': {'mass': 3.6, 'diaginertia': '0.022 0.022 0.010'},
    'forearm_link': {'mass': 1.3, 'diaginertia': '0.007 0.007 0.004'},
    'wrist_1_link': {'mass': 0.85, 'diaginertia': '0.0023 0.0021 0.0021'},
    'wrist_2_link': {'mass': 0.85, 'diaginertia': '0.0023 0.0021 0.0021'},
    'wrist_3_link': {'mass': 0.4, 'diaginertia': '0.001 0.0009 0.0009'},
}

for body_name, updates in bodies_to_update.items():
    body = root.xpath(f'//body[@name="{body_name}"]/inertial')[0]
    body.set('mass', str(updates['mass']))
    body.set('diaginertia', updates['diaginertia'])

# Create general type motors and specify name, joint, and ctrlrange
motors = [
    {'name': 'motor_shoulder_pan', 'joint': 'shoulder_pan_joint', 'ctrlrange': '-330 330'},
    {'name': 'motor_shoulder_lift', 'joint': 'shoulder_lift_joint', 'ctrlrange': '-330 330'},
    {'name': 'motor_elbow', 'joint': 'elbow_joint', 'ctrlrange': '-150 150'},
    {'name': 'motor_wrist_1', 'joint': 'wrist_1_joint', 'ctrlrange': '-54 54'},
    {'name': 'motor_wrist_2', 'joint': 'wrist_2_joint', 'ctrlrange': '-54 54'},
    {'name': 'motor_wrist_3', 'joint': 'wrist_3_joint', 'ctrlrange': '-54 54'},
]

actuator = etree.SubElement(root, 'actuator')
for motor in motors:
    motor_elem = etree.SubElement(actuator, 'motor')
    motor_elem.set('name', motor['name'])
    motor_elem.set('joint', motor['joint'])
    motor_elem.set('ctrlrange', motor['ctrlrange'])

# Save the resulting XML to a new file
tree.write('modified_mjmodel.xml', pretty_print=True, xml_declaration=True, encoding='utf-8')

print("XML model has been modified and saved to 'modified_mjmodel.xml'.")