#!/bin/bash
# Test script for Humble-Jazzy communication

set -e

echo "========================================"
echo "Testing Humble-Jazzy Communication"
echo "========================================"
echo ""
echo "This will start:"
echo "  - Humble container running talker node"
echo "  - Jazzy container running listener node"
echo ""
echo "Press Ctrl+C to stop the test"
echo ""

docker compose -f docker-compose.humble-jazzy.yml up --build
