# Installation Guide - Humanoid Robotics Backend

## Prerequisites

### 1. System Requirements

- **OS**: Ubuntu 20.04+ (WSL2 on Windows is supported)
- **Python**: 3.12+
- **RAM**: 2GB minimum
- **Disk Space**: 500MB minimum

### 2. Install Python and Dependencies

On Ubuntu/Debian/WSL:

```bash
# Update package list
sudo apt update

# Install Python 3.12 and venv
sudo apt install -y python3.12 python3.12-venv python3-pip

# Verify installation
python3 --version  # Should show 3.12.x
```

---

## Installation Steps

### Option 1: Automated Setup (Recommended)

```bash
# Navigate to backend directory
cd 1-docusaurus-textbook/backend

# Run setup script
./setup.sh
```

The script will:
1. Check Python version
2. Create virtual environment
3. Install all dependencies
4. Verify installation

### Option 2: Manual Setup

#### Step 1: Create Virtual Environment

```bash
cd 1-docusaurus-textbook/backend

# Create virtual environment
python3 -m venv venv

# Activate virtual environment
source venv/bin/activate  # Linux/Mac
# OR
venv\Scripts\activate  # Windows
```

#### Step 2: Install Dependencies

```bash
# Upgrade pip
pip install --upgrade pip

# Install all dependencies
pip install -r requirements.txt

# Verify installation
pip list | grep fastapi
```

#### Step 3: Configure Environment

The `.env` file is already created with development defaults.

```bash
# View current configuration
cat .env

# (Optional) Edit configuration
nano .env
```

---

## Verification

### 1. Check Installation

```bash
# Ensure virtual environment is activated
source venv/bin/activate

# Check Python packages
python -c "import fastapi; print('FastAPI:', fastapi.__version__)"
python -c "import uvicorn; print('Uvicorn:', uvicorn.__version__)"
python -c "import pydantic; print('Pydantic:', pydantic.__version__)"
```

Expected output:
```
FastAPI: 0.115.0
Uvicorn: 0.32.0
Pydantic: 2.10.0
```

### 2. Start the Server

```bash
# Method 1: Using run.py
python run.py

# Method 2: Using uvicorn directly
uvicorn app.main:app --reload
```

You should see:
```
INFO:     Uvicorn running on http://0.0.0.0:8000 (Press CTRL+C to quit)
INFO:     Started reloader process [12345] using StatReload
INFO:     Started server process [12346]
INFO:     Waiting for application startup.
🚀 Starting Humanoid Robotics Backend...
Environment: development
Debug Mode: True
Server: 0.0.0.0:8000
INFO:     Application startup complete.
```

### 3. Test Endpoints

**Terminal 1** (Server running):
```bash
python run.py
```

**Terminal 2** (Testing):
```bash
# Test root endpoint
curl http://localhost:8000/

# Test health check
curl http://localhost:8000/health

# Test ping
curl http://localhost:8000/ping
```

**Browser**:
- Visit: http://localhost:8000
- API Docs: http://localhost:8000/docs
- ReDoc: http://localhost:8000/redoc

### 4. Run Tests

```bash
# Ensure virtual environment is activated
source venv/bin/activate

# Run all tests
pytest

# Run with verbose output
pytest -v

# Run specific test file
pytest tests/test_health.py -v
```

Expected output:
```
================================ test session starts ================================
collected 5 items

tests/test_health.py::test_root_endpoint PASSED                              [ 20%]
tests/test_health.py::test_health_check PASSED                               [ 40%]
tests/test_health.py::test_ping PASSED                                       [ 60%]
tests/test_health.py::test_docs_available_in_debug PASSED                    [ 80%]
tests/test_health.py::test_cors_headers PASSED                              [100%]

================================= 5 passed in 0.45s =================================
```

---

## Troubleshooting

### Issue: `python3-venv` not available

**Error**:
```
The virtual environment was not created successfully because ensurepip is not available.
```

**Solution**:
```bash
sudo apt update
sudo apt install -y python3.12-venv
```

### Issue: `pip` not found

**Error**:
```
/usr/bin/python3: No module named pip
```

**Solution**:
```bash
sudo apt install -y python3-pip
```

### Issue: Port 8000 already in use

**Error**:
```
ERROR:    [Errno 98] Address already in use
```

**Solution**:
```bash
# Find process using port 8000
lsof -i :8000

# Kill the process
kill -9 <PID>

# OR use a different port
uvicorn app.main:app --port 8001
```

### Issue: Import errors after installation

**Error**:
```
ModuleNotFoundError: No module named 'fastapi'
```

**Solution**:
```bash
# Ensure virtual environment is activated
source venv/bin/activate

# Reinstall dependencies
pip install -r requirements.txt --force-reinstall
```

### Issue: Permission denied on `setup.sh`

**Solution**:
```bash
chmod +x setup.sh
./setup.sh
```

---

## Next Steps

After successful installation:

1. ✅ Verify all endpoints work
2. ✅ Run test suite
3. ✅ Review README.md for API documentation
4. ⏭️ Proceed to **Phase 2: Database Integration**

---

## Quick Reference

```bash
# Activate virtual environment
source venv/bin/activate

# Start server (development)
python run.py

# Start server (production-like)
uvicorn app.main:app --host 0.0.0.0 --port 8000

# Run tests
pytest

# Format code
black app/ tests/

# Lint code
ruff check app/ tests/

# Deactivate virtual environment
deactivate
```

---

## Support

If you encounter issues:

1. Check the [Troubleshooting](#troubleshooting) section
2. Review logs in the terminal
3. Verify Python version: `python3 --version`
4. Verify dependencies: `pip list`
5. Check `.env` configuration

---

**Installation Status**: ✅ Ready for Phase 1 Verification
