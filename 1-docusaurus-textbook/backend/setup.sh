#!/bin/bash
# Setup script for Humanoid Robotics Backend

set -e

echo "🚀 Setting up Humanoid Robotics Backend..."

# Check Python version
PYTHON_VERSION=$(python3 --version 2>&1 | awk '{print $2}')
echo "✓ Python version: $PYTHON_VERSION"

# Check if python3-venv is available
if ! python3 -m venv --help &> /dev/null; then
    echo "❌ python3-venv is not installed"
    echo "Please run: sudo apt install python3.12-venv"
    exit 1
fi

# Create virtual environment
if [ ! -d "venv" ]; then
    echo "📦 Creating virtual environment..."
    python3 -m venv venv
    echo "✓ Virtual environment created"
else
    echo "✓ Virtual environment already exists"
fi

# Activate virtual environment
echo "🔌 Activating virtual environment..."
source venv/bin/activate

# Upgrade pip
echo "⬆️  Upgrading pip..."
pip install --upgrade pip

# Install dependencies
echo "📥 Installing dependencies..."
pip install -r requirements.txt

echo ""
echo "✅ Setup complete!"
echo ""
echo "To start the server:"
echo "  1. Activate virtual environment: source venv/bin/activate"
echo "  2. Run the server: python run.py"
echo "  3. Visit: http://localhost:8000"
echo ""
