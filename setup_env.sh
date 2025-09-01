#!/bin/bash
# WiFi Radar environment setup

# Exit if any command fails
set -e

# Directory of this script
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

# Create requirements.txt if it doesn't exist
REQ_FILE="requirements.txt"
if [ ! -f "$REQ_FILE" ]; then
    cat > "$REQ_FILE" <<EOF
scapy==2.5.0
gpsd-py3==0.3.0
paho-mqtt==1.6.1
numpy==1.26.4
pyyaml==6.0.2
matplotlib==3.9.2
EOF
    echo "[i] requirements.txt created."
fi

# Create virtual environment if missing
if [ ! -d ".venv" ]; then
    echo "[i] Creating Python virtual environment..."
    python3 -m venv .venv
fi

# Activate virtual environment
echo "[i] Activating virtual environment..."
source .venv/bin/activate

# Upgrade pip and setuptools
echo "[i] Upgrading pip/setuptools..."
pip install --upgrade pip setuptools wheel

# Install dependencies
echo "[i] Installing required packages..."
pip install -r requirements.txt

echo "[i] Environment is ready. To activate later, run:"
echo "     source .venv/bin/activate"
