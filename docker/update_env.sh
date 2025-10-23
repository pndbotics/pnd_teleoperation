#!/bin/bash
# This script updates the .env file with the current user's UID and GID.
# Get current directory of the script
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

# Populate .env file
echo "UID=$(id -u)" > "${SCRIPT_DIR}/.env"
echo "GID=$(id -g)" >> "${SCRIPT_DIR}/.env"