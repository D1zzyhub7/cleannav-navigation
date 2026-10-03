import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.parameter import Parameter


rclpy.init()
node = rclpy.create_node('set_initial_pose_experiment')
node.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
publisher = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
message = PoseWithCovarianceStamped()
message.header.frame_id = 'map'
message.pose.pose.position.x = -1.875
message.pose.pose.position.y = 0.415
message.pose.pose.orientation.w = 1.0
message.pose.covariance[0] = 0.25
message.pose.covariance[7] = 0.25
message.pose.covariance[35] = 0.0685

# AMCL can be active before its subscription has finished discovery.  Keep
# publishing long enough to guarantee that the initial pose is received.
for _ in range(50):
    message.header.stamp = node.get_clock().now().to_msg()
    publisher.publish(message)
    rclpy.spin_once(node, timeout_sec=0.2)

print('initial pose published')
node.destroy_node()
rclpy.shutdown()
