#!/usr/bin/env python3
"""Read only servo position/torque registers; never enable or command motion.

Run only when no driver owns the serial port. Uses the existing direct-driver
protocol implementation, with an exclusive serial open.
"""
import argparse
import json
from pathlib import Path
import sys
import time
from project_paths import arm_source_path, default_arm_port


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--port',default=default_arm_port())
    ap.add_argument('--report',type=Path,required=True)
    args = ap.parse_args()
    sys.path.insert(0,arm_source_path())
    from so101_bringup.servo_protocol import FeetechSerialBus, REG_PRESENT_POSITION, REG_TORQUE_ENABLE
    result = dict(read_only=True,port=args.port,time=time.time(),servos=[])
    try:
        with FeetechSerialBus(args.port,timeout_s=.15) as bus:
            for servo_id in bus.IDS:
                item = dict(id=servo_id)
                for register,key,size in ((REG_PRESENT_POSITION,'position_raw',2),
                                          (REG_TORQUE_ENABLE,'torque_enabled',1)):
                    try:
                        data = bus.read_bytes(servo_id,register,size)
                        item[key] = int.from_bytes(data,'little')
                    except Exception as exc:
                        item[key+'_error'] = str(exc)
                result['servos'].append(item)
    except Exception as exc:
        result['open_error'] = str(exc)
    result['all_responded'] = len(result['servos']) == 6 and all('position_raw' in s and 'torque_enabled' in s for s in result['servos'])
    args.report.parent.mkdir(parents=True,exist_ok=True)
    args.report.write_text(json.dumps(result,indent=2))
    print(json.dumps(result),flush=True)
    return 0 if result['all_responded'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
