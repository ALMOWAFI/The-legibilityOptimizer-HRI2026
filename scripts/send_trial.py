#!/usr/bin/env python3
"""Manual planning/optimization request. Has no execution option."""
import argparse
import time

import rospy
from hri_legibility_preview.msg import BlockLayout, TrialRequest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trial-id', required=True)
    parser.add_argument('--target-id', type=int, required=True)
    parser.add_argument('--condition', choices=['baseline', 'legible'], default='legible')
    args = parser.parse_args(rospy.myargv()[1:])
    rospy.init_node('send_legibility_trial', anonymous=True)
    layout = rospy.wait_for_message('/hri/block_layout', BlockLayout, timeout=10)
    if args.target_id not in [block.id for block in layout.blocks]:
        raise SystemExit('target-id is absent from the published layout')
    pub = rospy.Publisher('/legibility/trial_request', TrialRequest, queue_size=1, latch=False)
    deadline = time.monotonic() + 10
    while pub.get_num_connections() == 0 and not rospy.is_shutdown():
        if time.monotonic() >= deadline:
            raise SystemExit('no planner subscribed to /legibility/trial_request')
        time.sleep(0.05)
    trial = TrialRequest()
    trial.header.stamp, trial.header.frame_id = rospy.Time.now(), layout.header.frame_id
    trial.trial_id, trial.target_id, trial.condition = args.trial_id, args.target_id, args.condition
    trial.layout_id = layout.layout_id
    pub.publish(trial)
    time.sleep(0.5)
    print('Trial published. Watch /legibility/status for acceptance or rejection; this is not an execution request.')


if __name__ == '__main__':
    main()
