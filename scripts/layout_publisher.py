#!/usr/bin/env python3
"""Keep the configured block layout available to late subscribers. No robot calls."""
import rospy
import yaml
from hri_legibility_preview import msg
from legibility_ros.layout import layout_from_config


def main():
    rospy.init_node('block_layout')
    with open(rospy.get_param('~experiment_config'), encoding='utf-8') as source:
        layout = layout_from_config(yaml.safe_load(source), msg)
    layout.header.stamp = rospy.Time.now()
    publisher = rospy.Publisher('/hri/block_layout', msg.BlockLayout, queue_size=1, latch=True)
    publisher.publish(layout)
    rospy.loginfo('Block layout %s available in %s; no motion triggered', layout.layout_id, layout.header.frame_id)
    rospy.spin()


if __name__ == '__main__':
    main()
