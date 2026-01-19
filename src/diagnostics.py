#!/usr/bin/env python3
"""
Diagnostic utilities for ROS2 cross-distribution communication testing.
Provides functions to print ROS2, rmw, and network configuration information.
"""

import os
import subprocess
import socket


def print_ros2_config():
    """Print ROS2 configuration information."""
    print("=" * 60)
    print("ROS2 Configuration")
    print("=" * 60)
    
    # ROS Distribution
    ros_distro = os.environ.get('ROS_DISTRO', 'unknown')
    print(f"ROS_DISTRO: {ros_distro}")
    
    # ROS Domain ID
    ros_domain_id = os.environ.get('ROS_DOMAIN_ID', 'not set')
    print(f"ROS_DOMAIN_ID: {ros_domain_id}")
    
    # ROS Localhost Only
    ros_localhost_only = os.environ.get('ROS_LOCALHOST_ONLY', 'not set')
    print(f"ROS_LOCALHOST_ONLY: {ros_localhost_only}")
    
    # ROS Version
    try:
        result = subprocess.run(
            ['ros2', '--version'],
            capture_output=True,
            text=True,
            timeout=5
        )
        if result.returncode == 0:
            print(f"ROS2 Version: {result.stdout.strip()}")
    except Exception as e:
        print(f"ROS2 Version: Unable to determine ({e})")
    
    print()


def print_rmw_config():
    """Print RMW (ROS Middleware) configuration information."""
    print("=" * 60)
    print("RMW Configuration")
    print("=" * 60)
    
    # RMW Implementation
    rmw_implementation = os.environ.get('RMW_IMPLEMENTATION', 'not set (using default)')
    print(f"RMW_IMPLEMENTATION: {rmw_implementation}")
    
    # Try to get available RMW implementations
    try:
        result = subprocess.run(
            ['ros2', 'doctor', '--report'],
            capture_output=True,
            text=True,
            timeout=10
        )
        if result.returncode == 0:
            # Extract RMW-related info from doctor report
            lines = result.stdout.split('\n')
            for i, line in enumerate(lines):
                if 'middleware' in line.lower() or 'rmw' in line.lower():
                    print(line)
                    # Print a few lines after for context
                    for j in range(i+1, min(i+3, len(lines))):
                        if lines[j].strip():
                            print(lines[j])
    except Exception as e:
        print(f"Unable to get detailed RMW info: {e}")
    
    # DDS-related environment variables
    dds_vars = [
        'CYCLONEDDS_URI',
        'FASTRTPS_DEFAULT_PROFILES_FILE',
        'RTI_LICENSE_FILE',
        'NDDS_QOS_PROFILES'
    ]
    
    print("\nDDS Environment Variables:")
    for var in dds_vars:
        value = os.environ.get(var, 'not set')
        print(f"  {var}: {value}")
    
    print()


def print_network_config():
    """Print network configuration information."""
    print("=" * 60)
    print("Network Configuration")
    print("=" * 60)
    
    # Hostname
    hostname = socket.gethostname()
    print(f"Hostname: {hostname}")
    
    # Get network interfaces
    try:
        result = subprocess.run(
            ['ip', 'addr', 'show'],
            capture_output=True,
            text=True,
            timeout=5
        )
        if result.returncode == 0:
            print("\nNetwork Interfaces:")
            # Parse and display relevant interface information
            lines = result.stdout.split('\n')
            current_iface = None
            for line in lines:
                if line and not line[0].isspace():
                    # Interface line
                    current_iface = line.split(':')[1].strip() if ':' in line else None
                    if current_iface and current_iface != 'lo':
                        print(f"\n  Interface: {current_iface}")
                elif 'inet ' in line and current_iface and current_iface != 'lo':
                    # IPv4 address
                    parts = line.strip().split()
                    if len(parts) >= 2:
                        print(f"    IPv4: {parts[1]}")
                elif 'inet6' in line and current_iface and current_iface != 'lo':
                    # IPv6 address
                    parts = line.strip().split()
                    if len(parts) >= 2 and 'scope global' in line:
                        print(f"    IPv6: {parts[1]}")
    except Exception as e:
        print(f"Unable to get network interface info: {e}")
    
    # Test basic connectivity
    print("\nNetwork Connectivity:")
    
    # Check if we can reach localhost
    try:
        result = subprocess.run(
            ['ping', '-c', '1', '-W', '1', '127.0.0.1'],
            capture_output=True,
            timeout=2
        )
        if result.returncode == 0:
            print("  Localhost (127.0.0.1): OK")
        else:
            print("  Localhost (127.0.0.1): FAILED")
    except Exception:
        print("  Localhost (127.0.0.1): Unable to test")
    
    # Multicast test (important for DDS discovery)
    print("\nMulticast Status:")
    try:
        result = subprocess.run(
            ['ip', 'maddr', 'show'],
            capture_output=True,
            text=True,
            timeout=5
        )
        if result.returncode == 0:
            print("  Multicast groups joined (showing non-lo interfaces):")
            lines = result.stdout.split('\n')
            current_iface = None
            for line in lines:
                if line and not line[0].isspace():
                    current_iface = line.split(':')[1].strip() if ':' in line else None
                    if current_iface and current_iface != 'lo':
                        print(f"\n    {current_iface}:")
                elif line.strip() and current_iface and current_iface != 'lo':
                    print(f"      {line.strip()}")
    except Exception as e:
        print(f"  Unable to get multicast info: {e}")
    
    print()


def print_container_info():
    """Print container-specific information."""
    print("=" * 60)
    print("Container Information")
    print("=" * 60)
    
    # Check if running in container
    in_container = os.path.exists('/.dockerenv')
    print(f"Running in Docker: {in_container}")
    
    if in_container:
        # Container hostname
        print(f"Container Hostname: {socket.gethostname()}")
        
        # Check network mode
        try:
            with open('/proc/1/cgroup', 'r') as f:
                cgroup_info = f.read()
                if 'docker' in cgroup_info:
                    print("Container Type: Docker")
        except Exception:
            pass
    
    print()


def print_all_diagnostics():
    """Print all diagnostic information."""
    print("\n" + "=" * 60)
    print("STARTING DIAGNOSTIC INFORMATION")
    print("=" * 60 + "\n")
    
    print_container_info()
    print_ros2_config()
    print_rmw_config()
    print_network_config()
    
    print("=" * 60)
    print("DIAGNOSTIC INFORMATION COMPLETE")
    print("=" * 60 + "\n")
