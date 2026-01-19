#!/bin/bash
# Test script for Foxy-Humble communication

set -e

echo "========================================"
echo "Testing Foxy-Humble Communication"
echo "========================================"
echo ""
echo "This will start:"
echo "  - Foxy container running talker node"
echo "  - Humble container running listener node"
echo ""
echo "Press Ctrl+C to stop the test"
echo ""

docker-compose -f docker-compose.foxy-humble.yml up --build
