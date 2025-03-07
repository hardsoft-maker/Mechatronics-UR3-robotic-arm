import lxml
from lxml import etree
import numpy as np

file_path = "C:/Users/smart/Downloads/mat/mjmodel.xml"
file_path_2 = "new_inertia.xml"
file_path_3 = "new_armature.xml"
reflected_inertia = 382e-7
gear_ratio = 101
allowable_torque = 30
parser = etree.XMLParser(remove_blank_text=True)


def str_to_arr(s):
    if s is not None:
        return np.array(s.split(' '), dtype='float64')
    else:
        return np.array([])


def arr_to_str(arr):
    return ' '.join([f'{i:.9f}' for i in arr])


def read_file(path):
    with open(file_path, 'r') as f:
        return etree.fromstringlist(f.readlines())
    

def add_actuators_sensors(tree):
    actuators = etree.SubElement(tree, 'actuator')
    sensors = etree.SubElement(tree, 'sensor')
    for joint in tree.findall('.//joint'):
        name = joint.get('name')
        etree.SubElement(actuators, 'position', attrib={
            'name': 'j_'+name,
            'joint': name,
            'kp': f'{0.01}',
            'dampratio': f'{1}',
            'gear': f'{gear_ratio}',
            'ctrlrange': arr_to_str([-np.pi*gear_ratio, np.pi*gear_ratio]),
            'forcerange': arr_to_str([-allowable_torque, allowable_torque])
        })
        etree.SubElement(sensors, 'jointpos', attrib={
            'name': 's_'+name,
            'joint': name
        })


tree_inertia = etree.parse(file_path, parser).getroot()
tree_inertia.find('.//compiler').set('meshdir', '../ur_description/meshes')
add_actuators_sensors(tree_inertia)

for body in tree_inertia.findall('.//body'):
    joint = body.find('joint')
    inertial = body.find('inertial')
    if joint is not None and inertial is not None:
        joint_axis = str_to_arr(joint.get('axis'))
        inertial_inertia = str_to_arr(inertial.get('diaginertia'))
        inertial_inertia = inertial_inertia + joint_axis * reflected_inertia
        inertial.set('diaginertia', arr_to_str(inertial_inertia))

with open(file_path_2, 'w') as f:
    f.write(etree.tostring(tree_inertia, pretty_print=True).decode())
        
tree_armature = etree.parse(file_path, parser).getroot()
tree_armature.find('.//compiler').set('meshdir', '../ur_description/meshes')
add_actuators_sensors(tree_armature)

for body in tree_armature.findall('.//body'):
    joint = body.find('joint')
    if joint is not None:
        joint.set('armature', f'{reflected_inertia:.9f}')

with open(file_path_3, 'w') as f:
    f.write(etree.tostring(tree_armature, pretty_print=True).decode())