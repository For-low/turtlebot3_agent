import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Quaternion
from action_msgs.msg import GoalStatus
from langchain.agents import tool
import threading
import time
from .locations import get_location_coords, get_closest_location_name, get_all_location_names
import math

# --- Shared Node Setup ---
_shared_node = None
_node_init_lock = threading.Lock()
_navigation_action_client = None

def initialize_node():
    global _shared_node, _navigation_action_client
    with _node_init_lock:
        if _shared_node is None:
            print("Initializing Shared ROS Node for Tools...")
            if not rclpy.ok():
                print("Warning: rclpy not initialized before tool node creation. Initializing now.")
                # Use try-except as context might already be initialized externally
                try:
                    rclpy.init()
                except Exception as init_e:
                    print(f"Note: rclpy init failed, likely already initialized: {init_e}")
            # Create the node, ensure it doesn't conflict if main node has same name base
            _shared_node = Node('turtlebot3_agent_tools_node', automatically_declare_parameters_from_overrides=True)
            print("Shared tools node created.")

            _navigation_action_client = ActionClient(_shared_node, NavigateToPose, '/navigate_to_pose')
            print("NavigateToPose action client created on shared node.")

def get_shared_node():
    if _shared_node is None:
        # This ensures initialization happens if called before main node fully sets up
        initialize_node()
    return _shared_node

def get_action_client():
    if _navigation_action_client is None:
        initialize_node()
    if _navigation_action_client is None:
        raise RuntimeError("Action client could not be initialized.")
    return _navigation_action_client
# --- End Shared Node Setup ---

def euler_to_quaternion(yaw, pitch=0.0, roll=0.0):
    """Converts Euler angles (in radians) to Quaternion."""
    cy = math.cos(yaw * 0.5); sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5); sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5); sr = math.sin(roll * 0.5)
    q = Quaternion()
    q.w = cy * cp * cr + sy * sp * sr
    q.x = cy * cp * sr - sy * sp * cr
    q.y = sy * cp * sr + cy * sp * cr
    q.z = sy * cp * cr - cy * sp * sr
    return q

@tool
def navigate_to_pose(x: float = None, y: float = None, yaw_degrees: float = 0.0, location_name: str = None) -> str:
    """
    Navigates the TurtleBot3 to a specified pose (x, y, yaw) or a named location using the Nav2 action server '/navigate_to_pose'.
    Provide *either* coordinate arguments (x, y, optional yaw_degrees) *or* the location_name argument.
    Yaw is in degrees relative to the map frame's X-axis (0 is +X, 90 is +Y, 180 is -X, -90 is -Y).

    :param x: Target x-coordinate in the 'map' frame. Used if location_name is None.
    :param y: Target y-coordinate in the 'map' frame. Used if location_name is None.
    :param yaw_degrees: Target orientation in degrees. Defaults to 0.0 if not specified when using coordinates. Ignored if location_name is provided.
    :param location_name: The name of a predefined location (e.g., 'kitchen', 'office'). Overrides x, y, yaw_degrees if provided.
    :return: A string indicating the outcome: 'Navigation successful: Reached goal near (x, y).', 'Navigation failed: Goal near (x, y) was aborted/canceled.', or an error message.
    """
    node = get_shared_node()
    action_client = get_action_client()
    node.get_logger().info(f"Navigate tool called with: x={x}, y={y}, yaw={yaw_degrees}, location='{location_name}'")

    target_x, target_y = None, None

    if location_name:
        target_location_name = get_closest_location_name(location_name)
        if not target_location_name:
            available_locations = ", ".join(get_all_location_names())
            err_msg = f"Error: Location name '{location_name}' not found or ambiguous. Available locations: {available_locations}"
            node.get_logger().error(err_msg)
            return err_msg
        coords = get_location_coords(target_location_name)
        if not coords:
             err_msg = f"Error: Could not retrieve coordinates for resolved location '{target_location_name}'."
             node.get_logger().error(err_msg)
             return err_msg
        target_x, target_y = coords
        node.get_logger().info(f"Resolved location: '{target_location_name}' to ({target_x:.2f}, {target_y:.2f})")
        # Use default yaw for named locations unless specified otherwise in future?
        target_yaw_degrees = 0.0
    elif x is not None and y is not None:
        target_x, target_y = float(x), float(y)
        target_yaw_degrees = float(yaw_degrees) if yaw_degrees is not None else 0.0
        node.get_logger().info(f"Navigating to coordinates: ({target_x:.2f}, {target_y:.2f}), Yaw: {target_yaw_degrees:.1f} deg")
    else:
        err_msg = "Error: Tool requires either (x, y) coordinates or a location_name."
        node.get_logger().error(err_msg)
        return err_msg

    # --- Action Client Logic ---
    server_wait_timeout = 5.0
    if not action_client.wait_for_server(timeout_sec=server_wait_timeout):
        err_msg = f"Error: Action server '/navigate_to_pose' not available after waiting {server_wait_timeout}s."
        node.get_logger().error(err_msg)
        return err_msg

    goal_msg = NavigateToPose.Goal()
    goal_msg.pose.header.stamp = node.get_clock().now().to_msg()
    goal_msg.pose.header.frame_id = 'map' # CRUCIAL
    goal_msg.pose.pose.position.x = target_x
    goal_msg.pose.pose.position.y = target_y
    goal_msg.pose.pose.position.z = 0.0 # Assume 2D

    yaw_radians = math.radians(target_yaw_degrees)
    goal_msg.pose.pose.orientation = euler_to_quaternion(yaw=yaw_radians)

    node.get_logger().info(f"Sending navigation goal to ({target_x:.2f}, {target_y:.2f})...")
    send_goal_future = action_client.send_goal_async(goal_msg)

    # --- Wait for Goal Acceptance ---
    goal_acceptance_timeout = 10.0
    try:
        # Use the shared node for spinning until future is complete
        rclpy.spin_until_future_complete(node, send_goal_future, timeout_sec=goal_acceptance_timeout)
    except Exception as e:
         err_msg = f"Error: Exception while waiting for goal acceptance: {e}"
         node.get_logger().error(err_msg)
         return err_msg

    if not send_goal_future.done() or send_goal_future.result() is None:
         err_msg = f"Error: Failed to send navigation goal (future not done or result is None after {goal_acceptance_timeout}s)."
         node.get_logger().error(err_msg)
         return err_msg

    goal_handle = send_goal_future.result()
    if not goal_handle.accepted:
        err_msg = "Error: Navigation goal was rejected by the action server."
        node.get_logger().error(err_msg)
        return err_msg

    goal_id_hex = bytes(goal_handle.goal_id.uuid).hex()
    node.get_logger().info(f"Navigation goal accepted (ID: {goal_id_hex}). Waiting for result...")

    # --- Wait for Result ---
    get_result_future = goal_handle.get_result_async()
    # Set a reasonable timeout for navigation (e.g., 5 minutes)
    navigation_timeout = 300.0
    try:
        rclpy.spin_until_future_complete(node, get_result_future, timeout_sec=navigation_timeout)
    except Exception as e:
         err_msg = f"Error: Exception while waiting for navigation result: {e}"
         node.get_logger().error(err_msg)
         # Consider attempting to cancel the goal here if possible
         # cancel_future = goal_handle.cancel_goal_async()
         # rclpy.spin_until_future_complete(node, cancel_future, timeout_sec=5.0)
         return err_msg

    if not get_result_future.done() or get_result_future.result() is None:
         err_msg = f"Error: Failed to get navigation result (future not done or result is None after {navigation_timeout}s)."
         node.get_logger().error(err_msg)
         # Consider cancellation here too
         return err_msg

    status = get_result_future.result().status
    # result_msg = get_result_future.result().result # Result is typically empty for NavigateToPose

    # --- Process Result ---
    if status == GoalStatus.STATUS_SUCCEEDED:
        node.get_logger().info("Navigation succeeded!")
        return f"Navigation successful: Reached goal near ({target_x:.2f}, {target_y:.2f})."
    elif status == GoalStatus.STATUS_ABORTED:
        node.get_logger().error("Navigation failed (aborted).")
        return f"Navigation failed: Goal near ({target_x:.2f}, {target_y:.2f}) was aborted by the server."
    elif status == GoalStatus.STATUS_CANCELED:
        node.get_logger().warn("Navigation canceled.")
        return f"Navigation canceled: Goal near ({target_x:.2f}, {target_y:.2f}) was canceled."
    else:
        node.get_logger().error(f"Navigation goal failed with status code: {status}")
        return f"Navigation failed: Goal near ({target_x:.2f}, {target_y:.2f}) ended with unknown status code {status}."
    # --- End Action Client Logic ---