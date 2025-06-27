#!/usr/bin/env python3
"""ROS 2 node that validates a gripper YAML configuration file and exposes
its keys as ROS parameters under the ``/gripper`` namespace.

The node is intentionally *read-only*: it runs once at start-up to ensure the
YAML file is present, syntactically correct and contains all mandatory
sections. If anything is missing it logs a fatal error and shuts down the
whole launch so that the user immediately notices the problem.

Mandatory sections
------------------
* ``piper_gripper`` – configuration for the on-arm ST3215 servo.
* ``gello_gripper`` – only required when the *gello_exist* launch argument is
  ``true``.

All keys (regardless of depth) are flattened into ``/gripper/<section>/<key>``
parameters so that the rest of the system can simply retrieve them with
``node.get_parameter('gripper/<section>/<key>')``.
"""
from __future__ import annotations

import os
import sys
from pathlib import Path
from typing import Any, Dict, List

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.duration import Duration
import yaml

MANDATORY_SECTIONS = ["piper_gripper"]


class GripperConfigLoader(Node):
    """Validates a YAML config and republishes it as ROS parameters."""

    def __init__(self) -> None:
        super().__init__("gripper_config_loader")

        # Required param – path to YAML file
        self.declare_parameter("gripper_config", rclpy.Parameter.Type.STRING)
        # Optional param – whether a Gello bridge is expected
        self.declare_parameter("gello_exist", True)

        # ------------------------------------------------------------------
        # Fetch and validate *gripper_config* argument
        # ------------------------------------------------------------------
        cfg_path: str = self.get_parameter("gripper_config").get_parameter_value().string_value
        if not cfg_path:
            self._fatal("Parameter 'gripper_config' is required and must point to a YAML file")
            return

        path = Path(cfg_path)
        if not path.is_file():
            self._fatal(f"YAML file does not exist: {path}")
            return

        try:
            with path.open("r", encoding="utf-8") as f:
                data: Dict[str, Any] = yaml.safe_load(f)
        except Exception as exc:  # broad catch so we definitely fail hard
            self._fatal(f"Failed to parse YAML '{path}': {exc}")
            return

        # ------------------------------------------------------------------
        # Structural validation
        # ------------------------------------------------------------------
        sections_required = list(MANDATORY_SECTIONS)
        if self.get_parameter("gello_exist").get_parameter_value().bool_value:
            sections_required.append("gello_gripper")

        missing_sections = [sec for sec in sections_required if sec not in data]
        if missing_sections:
            self._fatal(f"Missing YAML section(s): {', '.join(missing_sections)}")
            return

        # Add every key as ROS parameter
        for section, content in data.items():
            if not isinstance(content, dict):
                # Flatten scalar section as a single parameter
                full_name = f"gripper/{section}"
                self.declare_parameter(full_name, content)
                continue

            for key, value in content.items():
                full_name = f"gripper/{section}/{key}"
                self.declare_parameter(full_name, value)

        self.get_logger().info(f"Published gripper parameters from {path}")

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def _fatal(self, msg: str) -> None:
        """Log *msg* as FATAL and shut the whole graph down."""
        self.get_logger().fatal(msg)
        # Give the logger a tiny bit of time before bringing the system down
        try:
            self.get_clock().sleep_for(Duration(seconds=0.2))
        except Exception:
            pass
        rclpy.shutdown()

    @staticmethod
    def _infer_param_type(value: Any) -> Parameter.Type:
        """Return the ROS parameter type matching *value*."""
        if isinstance(value, bool):
            return Parameter.Type.BOOL
        if isinstance(value, int):
            return Parameter.Type.INTEGER
        if isinstance(value, float):
            return Parameter.Type.DOUBLE
        if isinstance(value, (list, tuple)):
            # Assume homogeneous list, map first element as type
            if not value:
                return Parameter.Type.STRING_ARRAY
            first = value[0]
            if isinstance(first, bool):
                return Parameter.Type.BOOL_ARRAY
            if isinstance(first, int):
                return Parameter.Type.INTEGER_ARRAY
            if isinstance(first, float):
                return Parameter.Type.DOUBLE_ARRAY
            return Parameter.Type.STRING_ARRAY
        # default to string
        return Parameter.Type.STRING


def main() -> None:  # pragma: no cover
    rclpy.init()
    loader = GripperConfigLoader()
    # If the node shut itself down due to an error *init* will already have
    # been finalised so we just return.
    if rclpy.ok():
        rclpy.spin(loader)
    rclpy.shutdown()


if __name__ == "__main__":
    main() 