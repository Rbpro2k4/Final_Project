#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class BatteryManager(Node):
    def __init__(self):
        super().__init__('battery_manager')
        self.waypoint_counter = 0
        self.battery_low = False
        self.publisher_ = self.create_publisher(String, 'battery_command', 10)
        self.subscription = self.create_subscription(
            String,
            'navigation_status',
            self.navigation_callback,
            10
        )
        self.get_logger().info("Battery Manager Node Started.")

    def navigation_callback(self, msg):
        if msg.data == "waypoint_reached":
            self.waypoint_counter += 1
            self.get_logger().info(f"Waypoint reached. Count: {self.waypoint_counter}")
            if self.waypoint_counter >= 4 and not self.battery_low:
                self.battery_low = True
                self.get_logger().info("Battery low! Navigating to charging station.")
                self.publish_command("navigate_to_charging_station")
                # Simulate recharging process
                self.recharge_battery()

    def publish_command(self, command):
        msg = String()
        msg.data = command
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published battery command: {command}")

    def recharge_battery(self):
        self.get_logger().info("Recharging... waiting for 30 seconds.")
        time.sleep(30)
        self.waypoint_counter = 0
        self.battery_low = False
        self.get_logger().info("Battery recharged. Resuming tasks.")
        self.publish_command("battery_recharged")

def main(args=None):
    rclpy.init(args=args)
    battery_manager = BatteryManager()
    rclpy.spin(battery_manager)
    battery_manager.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
