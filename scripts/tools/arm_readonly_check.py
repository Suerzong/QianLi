#!/usr/bin/env python3
"""Read six servo positions/torque flags without starting the direct driver."""
import argparse
import json
from pathlib import Path

from project_paths import default_arm_port, driver_params_path, project_path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', default=default_arm_port())
    parser.add_argument('--output', default=project_path('migration_assets/vm-acceptance/arm-readonly.json'))
    args = parser.parse_args()
    import yaml
    from so101_bringup.servo_protocol import FeetechSerialBus
    from arm_usb_watchdog import discover_usb_device
    info = discover_usb_device(args.port)
    report = dict(port=args.port, operations='READ only; no torque, position or EEPROM writes',
                  usb_discovery={key:str(value) for key,value in info.items()} if info else None,
                  servos=[])
    params = yaml.safe_load(Path(driver_params_path()).read_text())['so101_driver']['ros__parameters']
    try:
        with FeetechSerialBus(args.port, timeout_s=0.2) as bus:
            for index, servo_id in enumerate(bus.IDS):
                try:
                    position = bus.read_word(servo_id, 56)
                    torque = bus.read_byte(servo_id, 40)
                    report['servos'].append(dict(
                        id=servo_id, position=position, torque=torque,
                        within_limits=params['raw_min'][index] <= position <= params['raw_max'][index]))
                except Exception as exc:
                    report['servos'].append(dict(id=servo_id, error=repr(exc)))
        report['passed'] = all('position' in servo for servo in report['servos'])
    except Exception as exc:
        report.update(error=repr(exc), passed=False)
    destination = Path(args.output)
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps(report, indent=2))
    if not report['passed']:
        print('未读到全部舵机状态；请确认机械臂独立电源和串口连接。')
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
