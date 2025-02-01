from lxml import etree
import itertools

# Load the base XML model
tree = etree.parse('mjmodel.xml')
root = tree.getroot()

# Define harmonic drive variants for each joint
harmonic_drives = {
    'shoulder_pan_joint': [
        {'reduced_inertia': 0.005, 'max_torque': 350},
        {'reduced_inertia': 0.006, 'max_torque': 400},
    ],
    'shoulder_lift_joint': [
        {'reduced_inertia': 0.004, 'max_torque': 350},
        {'reduced_inertia': 0.005, 'max_torque': 400},
    ],
    'elbow_joint': [
        {'reduced_inertia': 0.003, 'max_torque': 160},
        {'reduced_inertia': 0.004, 'max_torque': 180},
    ],
    'wrist_1_joint': [
        {'reduced_inertia': 0.002, 'max_torque': 60},
        {'reduced_inertia': 0.0025, 'max_torque': 70},
    ],
    'wrist_2_joint': [
        {'reduced_inertia': 0.002, 'max_torque': 60},
        {'reduced_inertia': 0.0025, 'max_torque': 70},
    ],
    'wrist_3_joint': [
        {'reduced_inertia': 0.001, 'max_torque': 60},
        {'reduced_inertia': 0.0015, 'max_torque': 70},
    ],
}

# Generate all possible combinations of harmonic drives
combinations = list(itertools.product(*harmonic_drives.values()))

# Iterate over each combination and generate a new XML model
for i, combo in enumerate(combinations):
    # Create a copy of the base XML tree
    new_tree = etree.ElementTree(etree.fromstring(etree.tostring(root)))
    new_root = new_tree.getroot()

    # Update joints with the selected harmonic drive parameters
    for joint_name, variant in zip(harmonic_drives.keys(), combo):
        joint = new_root.xpath(f'//joint[@name="{joint_name}"]')[0]
        joint.set('armature', str(variant['reduced_inertia']))
        joint.set('actuatorfrcrange', f"{-variant['max_torque']} {variant['max_torque']}")

    # Save the new XML model to a file
    new_tree.write(f'mjmodel_variant_{i+1}.xml', pretty_print=True, xml_declaration=True, encoding='utf-8')

    print(f"Generated XML model for combination {i+1}: {combo}")

print("All XML models have been generated.")