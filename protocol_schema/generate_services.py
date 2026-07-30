#!/usr/bin/env python3
"""Generate the three language-specific V1 service policy tables."""

import argparse
import json
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
SCHEMA_PATH = Path(__file__).with_name('services.json')


def _entries(schema):
    for service in schema['services']:
        for opcode in service['opcodes']:
            yield service, opcode


def _render_python(schema):
    rows = []

    for service, opcode in _entries(schema):
        rows.append(
            '    '
            f'({service["id"]}, {opcode["id"]}, '
            f'QosClass.{opcode["qos"]}, '
            f'Priority.{opcode["priority"]}, '
            f'{opcode["deadline_ms"]}, '
            f'OverflowPolicy.{opcode["overflow"]}),'
        )

    return (
        '# flake8: noqa\n'
        '"""Generated from protocol_schema/services.json; do not edit."""\n\n'
        'from vision_demo.service_registry import (\n'
        '    OverflowPolicy,\n'
        '    Priority,\n'
        '    QosClass,\n'
        ')\n\n'
        'GENERATED_POLICY_ROWS = (\n'
        + '\n'.join(rows)
        + '\n)\n'
    )


def _render_cpp(schema):
    rows = []

    for service, opcode in _entries(schema):
        rows.append(
            '    {'
            f'{service["id"]}U, {opcode["id"]}U, '
            f'QosClass::{opcode["qos"]}, '
            f'Priority::{opcode["priority"]}, '
            f'{opcode["deadline_ms"]}U, '
            f'OverflowPolicy::{opcode["overflow"]}'
            '},'
        )

    return (
        '/* Generated from protocol_schema/services.json; do not edit. */\n'
        + '\n'.join(rows)
        + '\n'
    )


def _render_c(schema):
    rows = []

    for service, opcode in _entries(schema):
        rows.append(
            '    {'
            f'{service["id"]}U, {opcode["id"]}U, '
            f'ROBOT_QOS_{opcode["qos"]}, '
            f'ROBOT_PRIORITY_{opcode["priority"]}, '
            f'{opcode["deadline_ms"]}U, '
            f'ROBOT_OVERFLOW_{opcode["overflow"]}'
            '},'
        )

    return (
        '/* Generated from protocol_schema/services.json; do not edit. */\n'
        + '\n'.join(rows)
        + '\n'
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--check', action='store_true')
    arguments = parser.parse_args()
    schema = json.loads(SCHEMA_PATH.read_text(encoding='utf-8'))
    outputs = {
        ROOT / 'src/vision_demo/vision_demo/'
        'service_registry_generated.py': _render_python(schema),
        ROOT / 'ESP_control/esp32cam_gimbal/'
        'service_registry_generated.inc': _render_cpp(schema),
        ROOT / 'stm32_control/ros2/Core/Src/'
        'robot_service_registry_generated.inc': _render_c(schema),
    }
    stale = []

    for path, content in outputs.items():
        if arguments.check:
            if not path.exists() or path.read_text(
                encoding='utf-8'
            ) != content:
                stale.append(path)
            continue

        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content, encoding='utf-8')

    if stale:
        for path in stale:
            print(f'generated service registry is stale: {path}')
        return 1

    return 0


if __name__ == '__main__':
    sys.exit(main())
