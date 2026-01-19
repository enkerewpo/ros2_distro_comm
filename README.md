# ROS2 Distribution Communication Testing

This project tests inter-distribution communication between different ROS2 versions using Docker containers.

## Overview

This repository provides a test framework to validate communication between:
- **Foxy ↔ Humble**: Testing compatibility between ROS2 Foxy and Humble distributions
- **Humble ↔ Jazzy**: Testing compatibility between ROS2 Humble and Jazzy distributions

## Architecture

The project consists of:
- **Talker Node**: A publisher that sends messages to the `chatter` topic
- **Listener Node**: A subscriber that receives messages from the `chatter` topic
- **Docker Containers**: Separate containers for each ROS2 distribution (Foxy, Humble, Jazzy)

## Prerequisites

- Docker Engine (version 20.10 or higher)
- Docker Compose (version 1.29 or higher)

## Project Structure

```
ros2_distro_comm/
├── src/
│   ├── talker/
│   │   └── talker.py          # Publisher node
│   └── listener/
│       └── listener.py         # Subscriber node
├── docker/
│   ├── Dockerfile.foxy        # Dockerfile for ROS2 Foxy
│   ├── Dockerfile.humble      # Dockerfile for ROS2 Humble
│   └── Dockerfile.jazzy       # Dockerfile for ROS2 Jazzy
├── docker-compose.foxy-humble.yml    # Foxy-Humble test configuration
├── docker-compose.humble-jazzy.yml   # Humble-Jazzy test configuration
└── README.md
```

## Usage

### Test Foxy-Humble Communication

Run the following command to test communication between Foxy (talker) and Humble (listener):

```bash
docker-compose -f docker-compose.foxy-humble.yml up --build
```

**Expected behavior:**
- Foxy container publishes messages to the `chatter` topic
- Humble container subscribes and receives messages
- Console output shows successful cross-distribution communication

### Test Humble-Jazzy Communication

Run the following command to test communication between Humble (talker) and Jazzy (listener):

```bash
docker-compose -f docker-compose.humble-jazzy.yml up --build
```

**Expected behavior:**
- Humble container publishes messages to the `chatter` topic
- Jazzy container subscribes and receives messages
- Console output shows successful cross-distribution communication

### Stop the Containers

To stop the running containers:

```bash
# For Foxy-Humble test
docker-compose -f docker-compose.foxy-humble.yml down

# For Humble-Jazzy test
docker-compose -f docker-compose.humble-jazzy.yml down
```

## Technical Details

### Network Configuration

The containers use `network_mode: host` to enable direct communication between ROS2 nodes running in different containers. This allows DDS (Data Distribution Service) to discover nodes across containers.

### ROS Domain ID

All containers use `ROS_DOMAIN_ID=0` to ensure they communicate on the same DDS domain.

### Message Type

The test uses the standard `std_msgs/String` message type, which is compatible across all tested ROS2 distributions.

## Troubleshooting

### Containers cannot communicate

1. Ensure no firewall is blocking DDS traffic
2. Verify that `ROS_DOMAIN_ID` is the same for all containers
3. Check that containers are using `network_mode: host`

### Build failures

1. Ensure Docker has sufficient resources allocated
2. Check internet connection for downloading ROS2 base images
3. Verify Docker and Docker Compose versions

## Known Compatibility Issues

- Different ROS2 distributions may have varying levels of compatibility
- Some message types or features may not be backward/forward compatible
- Communication success depends on DDS implementation and configuration

## Contributing

Contributions are welcome! Please ensure all code and comments are written in English.

## License

This project is provided as-is for testing purposes.
