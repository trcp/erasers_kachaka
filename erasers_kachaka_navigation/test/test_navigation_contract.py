#!/usr/bin/env python3

from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
BT_PATH = PACKAGE_ROOT / 'behavior_trees' / 'kachaka_navigate_to_pose.xml'
PARAMS_PATH = PACKAGE_ROOT / 'params' / 'navigation.yaml'


def _rewrite_key(node, key, value):
    if isinstance(node, dict):
        return {
            child_key: (
                value
                if child_key == key
                else _rewrite_key(child_value, key, value)
            )
            for child_key, child_value in node.items()
        }
    if isinstance(node, list):
        return [_rewrite_key(child, key, value) for child in node]
    return node


def _values_for_key(node, key):
    values = []
    if isinstance(node, dict):
        for child_key, child_value in node.items():
            if child_key == key:
                values.append(child_value)
            values.extend(_values_for_key(child_value, key))
    elif isinstance(node, list):
        for child in node:
            values.extend(_values_for_key(child, key))
    return values


def test_behavior_tree_recovery_contract():
    root = ET.parse(BT_PATH).getroot()

    assert root.find('.//RateController').attrib['hz'] == '1.0'
    backups = root.findall('.//BackUp')
    assert len(backups) == 1
    assert backups[0].attrib == {
        'backup_dist': '0.10',
        'backup_speed': '0.04',
        'time_allowance': '4.0',
    }

    follow_recovery = next(
        node for node in root.findall('.//RecoveryNode')
        if node.attrib.get('name') == 'FollowPath'
    )
    assert [child.tag for child in follow_recovery] == [
        'FollowPath',
        'ClearEntireCostmap',
    ]
    assert follow_recovery.find('.//BackUp') is None

    fallback = next(
        node for node in root.findall('.//ReactiveFallback')
        if node.attrib.get('name') == 'SystemRecoveryFallback'
    )
    assert fallback.find('GoalUpdated') is not None
    recovery_actions = fallback.find('RoundRobin')
    assert [child.tag for child in recovery_actions] == [
        'Sequence',
        'Wait',
        'BackUp',
        'Spin',
    ]


def test_navigation_parameter_contract():
    params = yaml.safe_load(PARAMS_PATH.read_text(encoding='utf-8'))
    controller = params['controller_server']['ros__parameters']
    planner = params['planner_server']['ros__parameters']

    assert controller['failure_tolerance'] == 1.0
    assert controller['progress_checker']['required_movement_radius'] == 0.05
    assert controller['progress_checker']['movement_time_allowance'] == 6.0
    assert controller['general_goal_checker']['xy_goal_tolerance'] == 0.10
    assert controller['general_goal_checker']['yaw_goal_tolerance'] == 0.10
    assert controller['FollowPath']['xy_goal_tolerance'] == 0.10
    assert planner['expected_planner_frequency'] == 1.0
    assert planner['GridBased']['tolerance'] == 0.10


def test_prefix_and_use_sim_time_rewrites_are_complete():
    source = PARAMS_PATH.read_text(encoding='utf-8')

    for use_sim_time in (False, True):
        expanded_text = source.replace('{prefix}', 'test_robot_')
        expanded = yaml.safe_load(expanded_text)
        expanded = _rewrite_key(expanded, 'use_sim_time', use_sim_time)

        assert '{prefix}' not in expanded_text
        values = _values_for_key(expanded, 'use_sim_time')
        assert values
        assert all(value is use_sim_time for value in values)


def test_navigation_has_no_mode_specific_parameter_file():
    navigation_files = sorted(
        path.name for path in (PACKAGE_ROOT / 'params').glob('*nav*.yaml')
    )
    assert navigation_files == ['navigation.yaml']
