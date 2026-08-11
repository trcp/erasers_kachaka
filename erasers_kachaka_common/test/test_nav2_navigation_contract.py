#!/usr/bin/env python3

import ast
from pathlib import Path


SOURCE_PATH = (
    Path(__file__).resolve().parents[1]
    / 'erasers_kachaka_common'
    / 'navigator.py'
)
SOURCE = SOURCE_PATH.read_text(encoding='utf-8')
TREE = ast.parse(SOURCE)


def _class(name):
    return next(
        node for node in TREE.body
        if isinstance(node, ast.ClassDef) and node.name == name
    )


def _method(class_node, name):
    return next(
        node for node in class_node.body
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
        and node.name == name
    )


def _attribute_name(node):
    names = []
    while isinstance(node, ast.Attribute):
        names.append(node.attr)
        node = node.value
    if isinstance(node, ast.Name):
        names.append(node.id)
    return '.'.join(reversed(names))


def _succeeded_branch(move_abs):
    for node in ast.walk(move_abs):
        if not isinstance(node, ast.If):
            continue
        compare = node.test
        if not isinstance(compare, ast.Compare) or len(compare.comparators) != 1:
            continue
        if _attribute_name(compare.comparators[0]) == 'GoalStatus.STATUS_SUCCEEDED':
            return node
    raise AssertionError('STATUS_SUCCEEDED branch was not found')


def test_succeeded_branch_has_no_pid_or_publish():
    nav2 = _class('Nav2Navigation')
    branch = _succeeded_branch(_method(nav2, 'move_abs'))
    assigned_names = {
        target.id
        for node in ast.walk(branch)
        if isinstance(node, (ast.Assign, ast.AnnAssign))
        for target in (
            node.targets if isinstance(node, ast.Assign) else [node.target]
        )
        if isinstance(target, ast.Name)
    }
    calls = {
        _attribute_name(node.func)
        for node in ast.walk(branch)
        if isinstance(node, ast.Call)
    }

    assert assigned_names.isdisjoint({'KP', 'KI', 'KD'})
    assert not any(name.endswith('.publish') for name in calls)


def test_move_abs_keeps_angle_selection_and_releases_terminal_goal():
    nav2 = _class('Nav2Navigation')
    move_abs = _method(nav2, 'move_abs')
    succeeded_branch = _succeeded_branch(move_abs)
    status_assignment = next(
        node for node in ast.walk(move_abs)
        if isinstance(node, ast.Assign)
        and any(
            isinstance(target, ast.Name) and target.id == 'status'
            for target in node.targets
        )
    )

    assert any(
        isinstance(node, ast.If)
        and isinstance(node.test, ast.Name)
        and node.test.id == 'consider_angle'
        for node in ast.walk(move_abs)
    )
    goal_handle_clear = next(
        node for node in ast.walk(move_abs)
        if isinstance(node, ast.Assign)
        and status_assignment.lineno < node.lineno < succeeded_branch.lineno
        and isinstance(node.value, ast.Constant)
        and node.value.value is None
        and any(
            isinstance(target, ast.Attribute)
            and target.attr == '__current_goal_handle'
            for target in node.targets
        )
    )
    assert goal_handle_clear


def test_namespaced_tf_and_action_contract():
    assert "self.__frame_prefix = f'{namespace}_'" in SOURCE
    assert "self.__base_frame = f'{self.__frame_prefix}base_footprint'" in SOURCE
    assert 'f"/{namespace}/navigation/navigate_to_pose"' in SOURCE


def test_manual_motion_api_and_publisher_remain():
    nav2 = _class('Nav2Navigation')
    method_names = {
        node.name for node in nav2.body if isinstance(node, ast.FunctionDef)
    }

    assert 'move_forward' in method_names
    assert '__twist_publisher' in SOURCE
    assert any(
        isinstance(node, ast.Call)
        and _attribute_name(node.func).endswith('.publish')
        for node in ast.walk(_method(nav2, 'move_forward'))
    )
