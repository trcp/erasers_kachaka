#!/usr/bin/env python3
from erasers_kachaka_common.navigation import SimpleNavigator
import rclpy

def main():
    rclpy.init()
    sn = SimpleNavigator()
    sn.forward(0.1)

if __name__ == "__main__":
    main()