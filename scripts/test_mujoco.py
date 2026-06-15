#!/usr/bin/env python3
"""
Load a MuJoCo MJCF XML and open the interactive viewer.

Usage:
    python scripts/test_mujoco.py <path_to.xml>
"""

import argparse
import mujoco
import mujoco.viewer


def main():
    parser = argparse.ArgumentParser(description="Load MJCF and open MuJoCo viewer")
    parser.add_argument("xml", help="Path to the MuJoCo XML file")
    args = parser.parse_args()

    model = mujoco.MjModel.from_xml_path(args.xml)
    data = mujoco.MjData(model)

    mujoco.viewer.launch(model, data)


if __name__ == "__main__":
    main()
