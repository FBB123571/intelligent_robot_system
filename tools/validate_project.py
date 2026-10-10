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
from task_feedback import validate_feedback


def main():
    count = 0
    for directory in ('humanoid', 'turtlebot', 'tools', 'tests'):
        for path in (ROOT / directory).rglob('*.py'):
            ast.parse(path.read_text(encoding='utf-8'), filename=str(path))
            count += 1
    for path in (ROOT / 'turtlebot').rglob('*.launch'):
        ET.parse(str(path))
    for path in (ROOT / 'turtlebot').rglob('package.xml'):
        ET.parse(str(path))
    for path in list((ROOT / 'turtlebot').rglob('*.yaml')) + list((ROOT / 'humanoid/config').glob('*.yaml')):
        yaml.safe_load(path.read_text(encoding='utf-8'))
    config = yaml.safe_load((ROOT / 'humanoid/config/competition.yaml').read_text(encoding='utf-8'))
    validate_route(config)
    # Default ROIs deliberately remain uncalibrated; hardware checks enforce them.
    import copy
    feedback_check = copy.deepcopy(config)
    if feedback_check['feedback']['held_roi'] is None:
        feedback_check['feedback']['held_roi'] = [0, 0, .4, .4]
    if feedback_check['feedback']['placed_roi'] is None:
        feedback_check['feedback']['placed_roi'] = [.6, .6, 1, 1]
    validate_feedback(feedback_check)
    for path in (ROOT / 'humanoid/ActionGroup').rglob('*.d6a'):
        action_frames(path)
    print('Validated %d Python files, launch/package XML, YAML, route and d6a structure.' % count)
    print('Hardware calibration and ROS runtime are not validated by this script.')


if __name__ == '__main__':
    main()
