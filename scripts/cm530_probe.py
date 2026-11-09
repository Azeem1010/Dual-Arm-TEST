#!/usr/bin/env python3
"""Read-only protocol-5 probe: VERSION, PING, GET_HOME, READ. Never writes motor goals."""
import argparse
import json
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src/dual_arm_control'))
from dual_arm_control.cm530 import SerialCM530


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port',required=True)
    parser.add_argument('--timeout',type=float,default=2.0)
    parser.add_argument('--startup-listen',type=float,default=6.0)
    parser.add_argument('--output',type=Path)
    args=parser.parse_args()
    connection=SerialCM530(args.port,args.timeout,args.startup_listen)
    try:
        result={'protocol':5,'port':args.port,'baud':57600,'arms':{}}
        for arm in ('arm1','arm2'):
            result['arms'][arm]={'home_raw':connection.get_home(arm),'position_raw':connection.read(arm)}
        text=json.dumps(result,indent=2)
        if args.output:args.output.write_text(text+'\n')
        print(text)
    finally:
        connection.close()


if __name__=='__main__':main()
