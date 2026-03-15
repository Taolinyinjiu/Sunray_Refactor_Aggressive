#!/usr/bin/env python3

import threading

import rospy
from mavros_msgs.msg import RCIn


def trim_leading_slash(value):
    if value.startswith("/"):
        return value[1:]
    return value


def resolve_uav_ns():
    if rospy.has_param("~uav_ns"):
        return trim_leading_slash(str(rospy.get_param("~uav_ns")))
    if rospy.has_param("/uav_ns"):
        return trim_leading_slash(str(rospy.get_param("/uav_ns")))
    if rospy.has_param("/uav_name") and rospy.has_param("/uav_id"):
        return trim_leading_slash(
            str(rospy.get_param("/uav_name")) + str(rospy.get_param("/uav_id"))
        )
    return "uav1"


class MockRcInNode:
    def __init__(self):
        self.uav_ns = resolve_uav_ns()
        self.publish_hz = float(rospy.get_param("~publish_hz", 20.0))
        self.channel_count = max(8, int(rospy.get_param("~channel_count", 8)))
        self.default_pwm = int(rospy.get_param("~default_pwm", 1500))
        self.low_pwm = int(rospy.get_param("~low_pwm", 1100))
        self.mid_pwm = int(rospy.get_param("~mid_pwm", 1500))
        self.high_pwm = int(rospy.get_param("~high_pwm", 1900))
        self.switch_channel = max(1, int(rospy.get_param("~switch_channel", 7)))
        self.rssi = max(0, min(255, int(rospy.get_param("~rssi", 100))))

        default_topic = "/" + self.uav_ns + "/mavros/rc/in"
        self.output_topic = rospy.get_param("~output_topic", default_topic)
        self.channels = [self.default_pwm for _ in range(self.channel_count)]
        self.lock = threading.Lock()

        self.publisher = rospy.Publisher(self.output_topic, RCIn, queue_size=20)
        self.set_switch_level("2", announce=False)

        rospy.loginfo(
            "[mock_rc_in] started, topic=%s switch_channel=%d publish_hz=%.1f "
            "levels={1:%d,2:%d,3:%d}",
            self.output_topic,
            self.switch_channel,
            self.publish_hz,
            self.low_pwm,
            self.mid_pwm,
            self.high_pwm,
        )
        rospy.loginfo(
            "[mock_rc_in] commands: '1'/'2'/'3', 'set <channel> <pwm>', "
            "'show', 'quit'"
        )

        self.input_thread = threading.Thread(target=self.input_loop)
        self.input_thread.daemon = True
        self.input_thread.start()

    def publish_once(self):
        with self.lock:
            channels = list(self.channels)

        msg = RCIn()
        msg.header.stamp = rospy.Time.now()
        msg.rssi = self.rssi
        msg.channels = channels
        self.publisher.publish(msg)

    def set_channel_pwm(self, channel, pwm, announce=True):
        index = channel - 1
        if index < 0 or index >= self.channel_count:
            rospy.logwarn(
                "[mock_rc_in] channel out of range: %d, valid range=[1,%d]",
                channel,
                self.channel_count,
            )
            return False

        with self.lock:
            self.channels[index] = int(pwm)

        if announce:
            rospy.loginfo(
                "[mock_rc_in] channel %d set to pwm=%d", channel, int(pwm)
            )
        return True

    def set_switch_level(self, level, announce=True):
        if level == "1":
            pwm = self.low_pwm
        elif level == "2":
            pwm = self.mid_pwm
        elif level == "3":
            pwm = self.high_pwm
        else:
            rospy.logwarn("[mock_rc_in] unsupported level: %s", level)
            return False

        ok = self.set_channel_pwm(self.switch_channel, pwm, announce=False)
        if ok and announce:
            rospy.loginfo(
                "[mock_rc_in] switch channel %d -> level %s (pwm=%d)",
                self.switch_channel,
                level,
                pwm,
            )
        return ok

    def show_channels(self):
        with self.lock:
            channels = list(self.channels)
        rospy.loginfo("[mock_rc_in] channels=%s", channels)

    def input_loop(self):
        while not rospy.is_shutdown():
            try:
                line = input().strip()
            except EOFError:
                rospy.logwarn("[mock_rc_in] stdin closed, stop reading commands")
                return
            except Exception as exc:
                rospy.logwarn("[mock_rc_in] input error: %s", str(exc))
                return

            if not line:
                continue
            if line in ("1", "2", "3"):
                self.set_switch_level(line)
                continue
            if line == "show":
                self.show_channels()
                continue
            if line in ("q", "quit", "exit"):
                rospy.logwarn("[mock_rc_in] quit requested")
                rospy.signal_shutdown("mock rc quit")
                return

            parts = line.split()
            if len(parts) == 3 and parts[0] == "set":
                try:
                    channel = int(parts[1])
                    pwm = int(parts[2])
                except ValueError:
                    rospy.logwarn(
                        "[mock_rc_in] invalid set command, expected: set <channel> <pwm>"
                    )
                    continue
                self.set_channel_pwm(channel, pwm)
                continue

            rospy.logwarn(
                "[mock_rc_in] unknown command '%s', use 1/2/3, set <ch> <pwm>, show, quit",
                line,
            )


if __name__ == "__main__":
    rospy.init_node("mock_rc_in")
    node = MockRcInNode()
    rate = rospy.Rate(max(1.0, node.publish_hz))
    while not rospy.is_shutdown():
        node.publish_once()
        rate.sleep()
