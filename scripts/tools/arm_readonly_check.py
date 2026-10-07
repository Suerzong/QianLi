#!/usr/bin/env python3
"""Read six servo positions/torque flags without starting the direct driver."""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path

from project_paths import default_arm_port, driver_params_path, project_path

JOINT_NAMES = ('shoulder_pan', 'shoulder_lift', 'elbow_flex',
               'wrist_flex', 'wrist_roll', 'gripper')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', default=default_arm_port())
    parser.add_argument('--output', default=project_path('migration_assets/vm-acceptance/arm-readonly.json'))
    args = parser.parse_args()
    import yaml
    from so101_bringup.servo_protocol import FeetechSerialBus
    from arm_usb_watchdog import discover_usb_device
    info = discover_usb_device(args.port)
    report = dict(port=args.port, timestamp_utc=datetime.now(timezone.utc).isoformat(),
                  operations='READ only; no torque, position or EEPROM writes',
                  usb_discovery={key:str(value) for key,value in info.items()} if info else None,
                  servos=[])
    config = Path(driver_params_path())
    report['driver_config'] = str(config)
    params = yaml.safe_load(config.read_text(encoding='utf-8'))['so101_driver']['ros__parameters']
    try:
        with FeetechSerialBus(args.port, timeout_s=0.2) as bus:
            for index, servo_id in enumerate(bus.IDS):
                try:
                    position = bus.read_word(servo_id, 56)
                    torque = bus.read_byte(servo_id, 40)
                    report['servos'].append(dict(
                        id=servo_id, joint=JOINT_NAMES[index], position=position, torque=torque,
                        raw_min=params['raw_min'][index], raw_max=params['raw_max'][index],
                        within_limits=params['raw_min'][index] <= position <= params['raw_max'][index]))
                except Exception as exc:
                    report['servos'].append(dict(id=servo_id, joint=JOINT_NAMES[index], error=repr(exc)))
    except Exception as exc:
        report['error'] = repr(exc)
    report['communication_ok'] = (len(report['servos']) == len(JOINT_NAMES)
                                   and all('position' in servo for servo in report['servos']))
    report['within_joint_limits'] = (report['communication_ok']
                                     and all(servo['within_limits'] for servo in report['servos']))
    report['torque_disabled'] = (report['communication_ok']
                                 and all(servo['torque'] == 0 for servo in report['servos']))
    report['passed'] = (report['communication_ok'] and report['within_joint_limits']
                        and report['torque_disabled'])
    destination = Path(args.output)
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps(report, indent=2))
    if not report['communication_ok']:
        print('未读到全部舵机状态；请确认机械臂独立电源和串口连接。')
    else:
        if not report['within_joint_limits']:
            print('舵机通信正常，但有关节超出软限位；本次只读验收未通过。')
        if not report['torque_disabled']:
            print('检测到舵机扭矩开启；本工具只读取状态，不会关闭扭矩。')
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
