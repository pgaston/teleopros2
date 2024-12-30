class JetBotControlNode:
    def __init__(self):
        # Initialize the ROS2 node
        self.node = rclpy.create_node('jetbot_control_node')
        
        # Create publishers and subscribers
        self.publisher = self.node.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription = self.node.create_subscription(
            Twist, 'cmd_vel', self.listener_callback, 10)
        
    def listener_callback(self, msg):
        # Handle incoming messages
        self.process_command(msg)

    def process_command(self, msg):
        # Process the command and publish to the robot
        self.publisher.publish(msg)

    def spin(self):
        # Spin the node to keep it active
        rclpy.spin(self.node)

    def shutdown(self):
        # Clean up on shutdown
        self.node.destroy_node()
        rclpy.shutdown()