import lxml
from lxml import etree
import numpy as np
from scipy.spatial.transform import Rotation


class Motor:
    def __init__(self, name, reflected_inertia, gear_ratio, allowable_torque, length, d, mass):
        self.name = name
        self.reflected_inertia = reflected_inertia
        self.gear_ratio = gear_ratio
        self.allowable_torque = allowable_torque
        self.length = length
        self.d = d
        self.mass = mass
        self.inertia_main = mass * (d/2)**2 / 2 # diagonal element of inertia matrix along actuation axis
        self.inertia_sec = mass * (3*(d/2)**2 + length**2) / 12 # other diagonal elements of inertia matrix


file_path = "C:/Users/smart/Downloads/mat/mjmodel.xml"
# chosen motors
motors_data = [
    Motor('CRA-RI80-110-PRO-101', 0.00001255, 101, 2.3, 0.050, 0.050, 0.18),
    Motor('CRA-RI40-52-PRO-101', 82e-7, 101, 8.9, 0.060, 0.060, 0.3),
    Motor('CRA-RI50-70-NH-101', 124e-7, 101, 11, 0.060, 0.060, 0.63)
]
parser = etree.XMLParser(remove_blank_text=True)


def str_to_arr(s, default=None):
    if s is not None:
        return np.array(s.split(' '), dtype='float64')
    else:
        return default

def arr_to_str(arr):
    return ' '.join([f'{i:.9f}' for i in arr])

def read_file(path):
    with open(file_path, 'r') as f:
        return etree.fromstringlist(f.readlines())
    

def add_actuators_sensors(tree, motor: Motor):
    actuators = etree.SubElement(tree, 'actuator')
    sensors = etree.SubElement(tree, 'sensor')
    for joint in tree.findall('.//joint'):
        name = joint.get('name')
        etree.SubElement(actuators, 'position', attrib={
            'name': 'j_'+name,
            'joint': name,
            'kp': f'{0.01}', # set empirically
            'dampratio': f'{1}',
            'gear': f'{motor.gear_ratio}',
            'ctrlrange': arr_to_str([-np.pi*motor.gear_ratio, np.pi*motor.gear_ratio]),
            'forcerange': arr_to_str([-motor.allowable_torque, motor.allowable_torque])
        })
        etree.SubElement(sensors, 'jointpos', attrib={
            'name': 's_'+name,
            'joint': name
        })


for motor in motors_data:
    tree = etree.parse(file_path, parser).getroot()
    tree.find('.//compiler').set('meshdir', '../ur_description/meshes')
    add_actuators_sensors(tree, motor)

    for joint in tree.findall('.//joint'):
        joint_body = joint.getparent()
        parent_body = joint_body.getparent()
        inertial = parent_body.find('inertial')

        # i checked and seems like axis of joint in new body is same as it is in parent body
        # (some are inversed, but we don't need that info)
        mot_axis = str_to_arr(joint.get('axis'))
        jb_pos = str_to_arr(joint_body.get('pos'))
        # assume position of motor as position of new body except actuation axis
        mot_pos = jb_pos * (1 - mot_axis)
        inertial_diaginertia = str_to_arr(inertial.get('diaginertia'))
        inertial_pos = str_to_arr(inertial.get('pos'))
        inertial_mass = float(inertial.get('mass'))
        
        new_mass = inertial_mass + motor.mass
        new_pos = (inertial_pos * inertial_mass + mot_pos * motor.mass) / new_mass
        d_idi = inertial_pos - new_pos
        d_mdi = mot_pos - new_pos
        motor_diaginertia = mot_axis * motor.inertia_main + (1 - mot_axis) * motor.inertia_sec
        new_diaginertia = inertial_diaginertia + motor_diaginertia + \
            inertial_mass * (d_idi**2)[[[1, 2], [0, 2], [0, 1]]].sum(axis=1) + \
            motor.mass * (d_mdi**2)[[[1, 2], [0, 2], [0, 1]]].sum(axis=1)
        
        inertial.set('pos', arr_to_str(new_pos))
        inertial.set('mass', str(new_mass))
        inertial.set('diaginertia', arr_to_str(new_diaginertia))

        joint.set('armature', f'{motor.reflected_inertia:.9f}')

    with open(f'actuator_sizing/UR3_{motor.name}.xml', 'w') as f:
        f.write(etree.tostring(tree, pretty_print=True).decode())