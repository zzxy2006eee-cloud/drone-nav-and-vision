#!/usr/bin/env python3
"""Derive the inspection SITL airframe from PX4 v1.15 Iris dynamics."""
from pathlib import Path
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
PX4 = ROOT.parents[1] / 'external' / 'PX4-Autopilot'
IRIS = (PX4 / 'Tools/simulation/gazebo-classic/sitl_gazebo-classic/'
        'models/iris/iris.sdf.jinja')
DEST = ROOT / 'models/inspection_quad'


def sub(parent, tag, value=None, **attributes):
    node = ET.SubElement(parent, tag, attributes)
    if value is not None:
        node.text = str(value)
    return node


def indent(node, depth=0):
    if len(node):
        if not node.text or not node.text.strip():
            node.text = '\n' + '  ' * (depth + 1)
        for child in node:
            indent(child, depth + 1)
        if not child.tail or not child.tail.strip():
            child.tail = '\n' + '  ' * depth
    if depth and (not node.tail or not node.tail.strip()):
        node.tail = '\n' + '  ' * depth


def camera(link, name, pose, camera_pose):
    sensor = sub(link, 'sensor', name=name, type='camera')
    sub(sensor, 'pose', pose)
    sub(sensor, 'always_on', 'true')
    sub(sensor, 'update_rate', '15')
    optic = sub(sensor, 'camera', name=name)
    sub(optic, 'horizontal_fov', '1.3962634')
    image = sub(optic, 'image')
    sub(image, 'width', '640')
    sub(image, 'height', '480')
    sub(image, 'format', 'R8G8B8')
    clip = sub(optic, 'clip')
    sub(clip, 'near', '0.05')
    sub(clip, 'far', '35')
    plugin = sub(sensor, 'plugin', name=name + '_ros', filename='libgazebo_ros_camera.so')
    sub(plugin, 'robotNamespace', '/')
    sub(plugin, 'alwaysOn', 'true')
    sub(plugin, 'updateRate', '0')
    sub(plugin, 'cameraName', '/drone/' + name.split('_')[0])
    sub(plugin, 'imageTopicName', 'image_raw')
    sub(plugin, 'cameraInfoTopicName', 'camera_info')
    sub(plugin, 'frameName', name + '_optical')
    # camera_pose is represented by an independent ROS TF static transform.
    assert camera_pose


def main():
    tree = ET.parse(IRIS)
    sdf = tree.getroot()
    model = sdf.find('model')
    model.set('name', 'inspection_quad')
    for element in list(model):
        if (element.tag == 'include' and element.findtext('uri') == 'model://gps') or (
            element.tag == 'joint' and element.get('name') == 'gps0_joint') or (
            element.tag == 'link' and element.get('name') == '/imu_link') or (
            element.tag == 'joint' and element.get('name') == '/imu_joint') or (
            element.tag == 'plugin' and element.get('name') in
            ('magnetometer_plugin', 'barometer_plugin', 'groundtruth_plugin')):
            model.remove(element)

    imu_plugin = model.find("plugin[@name='rotors_gazebo_imu_plugin']")
    imu_plugin.find('linkName').text = 'base_link'

    base = model.find("link[@name='base_link']")
    # Gazebo's IMU sensor computes specific force correctly while the vehicle
    # rests on the floor; the legacy Rotors link acceleration does not.
    fcu_imu = sub(base, 'sensor', name='px4_imu', type='imu')
    sub(fcu_imu, 'pose', '0 0 0.02 0 0 0')
    sub(fcu_imu, 'always_on', 'true')
    sub(fcu_imu, 'update_rate', '250')
    # The Iris mesh underside is about 0.055 m below base_link. Landing feet
    # extend to -0.17 m, matching the agreed ground clearance.
    for index, (x, y) in enumerate(((0.19, 0.19), (0.19, -0.19),
                                     (-0.19, 0.19), (-0.19, -0.19))):
        for kind in ('collision', 'visual'):
            foot = sub(base, kind, name='leg_%d_%s' % (index, kind))
            sub(foot, 'pose', '%s %s -0.105 0 0 0' % (x, y))
            geom = sub(foot, 'geometry')
            cylinder = sub(geom, 'cylinder')
            sub(cylinder, 'radius', '0.012')
            sub(cylinder, 'length', '0.13')
            if kind == 'collision':
                # Compliant landing feet dissipate impact over several HIL
                # samples instead of generating a one-tick rigid bounce.
                surface = sub(foot, 'surface')
                bounce = sub(surface, 'bounce')
                sub(bounce, 'restitution_coefficient', '0')
                sub(bounce, 'threshold', '0.01')
                contact = sub(sub(surface, 'contact'), 'ode')
                sub(contact, 'kp', '1000')
                sub(contact, 'kd', '40')
                sub(contact, 'max_vel', '0.1')
                sub(contact, 'min_depth', '0.001')

    housing = sub(base, 'visual', name='mid360_housing')
    sub(housing, 'pose', '0.27 0 0.10 0 0.5235987756 0')
    geom = sub(housing, 'geometry')
    cylinder = sub(geom, 'cylinder')
    sub(cylinder, 'radius', '0.055')
    sub(cylinder, 'length', '0.08')

    sensor = sub(base, 'sensor', name='mid360_sim', type='ray')
    sub(sensor, 'pose', '0.27 0 0.10 0 0.5235987756 0')
    sub(sensor, 'always_on', 'true')
    sub(sensor, 'update_rate', '10')
    ray = sub(sensor, 'ray')
    scan = sub(ray, 'scan')
    horizontal = sub(scan, 'horizontal')
    sub(horizontal, 'samples', '360')
    sub(horizontal, 'resolution', '1')
    sub(horizontal, 'min_angle', '-3.14159265')
    sub(horizontal, 'max_angle', '3.14159265')
    vertical = sub(scan, 'vertical')
    sub(vertical, 'samples', '56')
    sub(vertical, 'resolution', '1')
    sub(vertical, 'min_angle', '-0.1221730476')
    sub(vertical, 'max_angle', '0.9075712110')
    rng = sub(ray, 'range')
    sub(rng, 'min', '0.1')
    sub(rng, 'max', '40')
    sub(rng, 'resolution', '0.02')
    # Native LaserScanStamped feeds the C++ frontend; no old block-laser plugin.

    imu = sub(base, 'sensor', name='mid360_imu', type='imu')
    sub(imu, 'pose', '0.27 0 0.10 0 0.5235987756 0')
    sub(imu, 'always_on', 'true')
    sub(imu, 'update_rate', '200')
    plugin = sub(imu, 'plugin', name='mid360_ros_imu', filename='libgazebo_ros_imu_sensor.so')
    sub(plugin, 'robotNamespace', '/')
    sub(plugin, 'topicName', '/drone/sim/lidar/imu_raw')
    sub(plugin, 'bodyName', 'base_link')
    sub(plugin, 'frameName', 'lidar')
    sub(plugin, 'updateRateHZ', '200')
    sub(plugin, 'gaussianNoise', '0.001')

    # Keep the lens outside the Iris body (collision half-width 0.235 m),
    # otherwise the camera renders the inside of the vehicle mesh.
    camera(base, 'front_camera', '0.27 0 0.05 0 0 0', 'front')
    camera(base, 'down_camera', '0 0 -0.15 0 1.5707963268 0', 'down')

    DEST.mkdir(parents=True, exist_ok=True)
    indent(sdf)
    template = DEST / 'inspection_quad.sdf.jinja'
    tree.write(template, encoding='unicode', xml_declaration=True)
    (DEST / 'model.config').write_text(
        '<?xml version="1.0"?><model><name>inspection_quad</name>'
        '<version>1.0</version><sdf version="1.6">inspection_quad.sdf</sdf>'
        '<author><name>project0</name></author>'
        '<description>PX4 inspection quad</description></model>')
    print(template)


if __name__ == '__main__':
    main()
