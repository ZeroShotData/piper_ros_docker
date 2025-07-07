#!/usr/bin/env python3
"""Guard node that ensures only one publisher exists on /joint_ctrl.

If more than one publisher is detected it logs an error and exits the
process so that docker/launch supervision can restart the whole stack.
"""

import sys
import rclpy
from rclpy.node import Node


class SinglePublisherGuard(Node):
    """Terminates the process when more than one publisher is detected."""

    def __init__(self) -> None:
        super().__init__("single_publisher_guard")
        # Check every second
        self._timer = self.create_timer(1.0, self._on_timer)

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def _on_timer(self) -> None:
        publishers = self.get_publishers_info_by_topic("/joint_ctrl")
        if len(publishers) > 1:
            self.get_logger().error(
                "Multiple publishers detected on /joint_ctrl; shutting down."
            )
            # Shutdown the ROS graph first so that other nodes are notified
            rclpy.shutdown()
            # Then exit so any supervising launch restarts us as needed
            sys.exit(1)


# ----------------------------------------------------------------------
# Main entry point
# ----------------------------------------------------------------------

def main() -> None:  # noqa: D401 – simple main function
    rclpy.init()
    node = SinglePublisherGuard()
    rclpy.spin(node)

    # Clean up (usually unreachable because of sys.exit when error occurs)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main() 