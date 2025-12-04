#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from ros_gz_interfaces.msg import Contacts
from geometry_msgs.msg import WrenchStamped


LEG_TOPICS = {
    "FL": "/visual/FL_foot_contact/the_force",
    "FR": "/visual/FR_foot_contact/the_force",
    "RL": "/visual/RL_foot_contact/the_force",
    "RR": "/visual/RR_foot_contact/the_force",
}


class TestBlockContactsToWrench(Node):
    def __init__(self):
        super().__init__("test_block_contacts_to_wrench")

        # Subscribe to the bridged test_block contact sensor
        self.sub = self.create_subscription(
            Contacts,
            "/world/default/model/test_block/link/link/sensor/contact_sensor/contact",
            self.contacts_callback,
            10,
        )

        # One publisher per leg
        self.pub = {
            leg: self.create_publisher(WrenchStamped, topic, 10)
            for leg, topic in LEG_TOPICS.items()
        }

        self.get_logger().info("TestBlockContactsToWrench node started")

    def contacts_callback(self, msg: Contacts):
        # Init forces for each leg
        forces = {
            "FL": [0.0, 0.0, 0.0],
            "FR": [0.0, 0.0, 0.0],
            "RL": [0.0, 0.0, 0.0],
            "RR": [0.0, 0.0, 0.0],
        }

        # Loop over all contacts in this message
        for c in msg.contacts:
            # collision1 = test_block, collision2 = robot link
            name = c.collision2.name

            leg = None
            if "FL_" in name:
                leg = "FL"
            elif "FR_" in name:
                leg = "FR"
            elif "RL_" in name:
                leg = "RL"
            elif "RR_" in name:
                leg = "RR"

            if leg is None:
                # Not a leg contact, skip
                continue

            # 🔧 FIX HERE: "wrenches" instead of "wrench"
            for w in c.wrenches:
                # We want force on the robot (collision2)
                f = w.body_2_wrench.force
                forces[leg][0] -= f.x
                forces[leg][1] -= f.y
                forces[leg][2] -= f.z

        # Publish per-leg WrenchStamped
        now = self.get_clock().now().to_msg()
        for leg, vec in forces.items():
            msg_out = WrenchStamped()
            msg_out.header.stamp = now
            msg_out.header.frame_id = f"{leg}_foot"   # e.g. RL_foot, FR_foot
            msg_out.wrench.force.x = vec[0]
            msg_out.wrench.force.y = vec[1]
            msg_out.wrench.force.z = vec[2]
            # torque left at 0
            self.pub[leg].publish(msg_out)


def main(args=None):
    rclpy.init(args=args)
    node = TestBlockContactsToWrench()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

