#!/usr/bin/env python3
"""Static validation of new scripts, YAML, XML and hardware action files."""
import ast
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'humanoid/scripts'))
from competition import action_frames
from competition_core.mission import validate_route
sys.path.insert(0, str(ROOT / 'turtlebot/follow3/rgbd_following/src'))
from rgbd_following_core.settings import validate as validate_rgbd


def main():
    count = 0
    for directory in ('humanoid', 'turtlebot', 'tools', 'tests'):
        for path in (ROOT / directory).rglob('*.py'):
            ast.parse(path.read_text(encoding='utf-8'), filename=str(path))
            count += 1
    for path in list((ROOT / 'turtlebot').rglob('*.launch')) + list((ROOT / 'turtlebot').rglob('*.test')):
        ET.parse(str(path))
    for path in (ROOT / 'turtlebot').rglob('package.xml'):
        ET.parse(str(path))
    for path in list((ROOT / 'turtlebot').rglob('*.yaml')) + list((ROOT / 'humanoid/config').glob('*.yaml')):
        yaml.safe_load(path.read_text(encoding='utf-8'))
    config = yaml.safe_load((ROOT / 'humanoid/config/competition.yaml').read_text(encoding='utf-8'))
    validate_route(config)
    validate_rgbd(yaml.safe_load((ROOT / 'turtlebot/follow3/rgbd_following/config/follow.yaml').read_text(encoding='utf-8')))
    navigation = yaml.safe_load((ROOT / 'turtlebot/follow3/rgbd_following/config/navigation.yaml').read_text(encoding='utf-8'))
    dwa = navigation['DWAPlannerROS']
    if (dwa['min_trans_vel'] > dwa['acc_lim_x'] / navigation['controller_frequency'] or
            dwa['min_rot_vel'] > dwa['acc_lim_theta'] / navigation['controller_frequency']):
        raise ValueError('DWA minimum speed must be reachable from rest in one control window')
    for path in (ROOT / 'humanoid/ActionGroup').rglob('*.d6a'):
        action_frames(path)
    print('Validated %d Python files, launch/package XML, YAML, route and d6a structure.' % count)
    print('Hardware calibration and ROS runtime are not validated by this script.')


if __name__ == '__main__':
    main()
