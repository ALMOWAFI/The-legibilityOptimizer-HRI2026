#!/usr/bin/env python3
"""Check topic connections, send one planning-only trial, and verify actual output."""
import argparse
import sys

import rosgraph
import rospy
from std_msgs.msg import String
from hri_legibility_preview import msg

from legibility_ros.flow_check import FlowCheck


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--target-id', type=int, default=0)
    parser.add_argument('--condition', choices=['baseline', 'legible'], default='legible')
    parser.add_argument('--timeout', type=float, default=180., help='seconds to wait for a trial result')
    parser.add_argument('--ready-timeout', type=float, default=20.)
    parser.add_argument('--node-namespace', default='/legibility_preview')
    args = parser.parse_args(rospy.myargv()[1:])
    rospy.init_node('check_legibility_pipeline', anonymous=True)
    master = rosgraph.Master(rospy.get_name())
    checker = FlowCheck(rospy, msg, String,
                        lambda: (master.getSystemState(), master.getTopicTypes()),
                        report=lambda line: print(line, flush=True), namespace=args.node_namespace)
    try:
        checker.run(args.target_id, args.condition, args.timeout, args.ready_timeout)
        return 0
    except (RuntimeError, ValueError, rospy.ROSException) as error:
        print('FAIL: ' + str(error), file=sys.stderr, flush=True)
        return 1


if __name__ == '__main__':
    sys.exit(main())
