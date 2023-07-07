#!/usr/bin/env python3

import rospy
from tf import TransformListener


class GestureDetector():
    
    def __init__(self) -> None:
        super().__init__()
        rospy.init_node("gestures", anonymous=True)

        self.transform_listener = TransformListener()
        self.tfs = []

    def _update(self) -> None:
        tfs = self.transform_listener

    def launch(self) -> None:
        """Continuously listen for tf data, and publish gestures."""

        try:
            while not rospy.is_shutdown():
                self._update()
        except KeyboardInterrupt:
            rospy.signal_shutdown("KeyboardInterrupt")
            raise SystemExit