#!/usr/bin/env python3
"""
Setup verification script for skills and subagents framework.

Verifies that the project structure is correct and all imports work.
Run this after initial setup to ensure everything is configured properly.
"""

import sys
from pathlib import Path


def verify_structure():
    """Verify project directory structure."""
    print("Verifying project structure...")

    required_dirs = [
        "src",
        "src/models",
        "src/services",
        "src/integrations",
        "src/integrations/ros2",
        "src/cli",
        "src/lib",
        "tests",
        "tests/unit",
        "tests/integration",
        "tests/fixtures",
        "examples",
    ]

    missing = []
    for dir_path in required_dirs:
        if not Path(dir_path).is_dir():
            missing.append(dir_path)

    if missing:
        print(f"  ✗ Missing directories: {', '.join(missing)}")
        return False

    print("  ✓ All required directories present")
    return True


def verify_imports():
    """Verify that all packages can be imported."""
    print("\nVerifying package imports...")

    try:
        import src
        print(f"  ✓ src package v{src.__version__}")

        from src import models, services, integrations, cli, lib
        print("  ✓ All submodules import successfully")

        return True
    except ImportError as e:
        print(f"  ✗ Import failed: {e}")
        return False


def verify_config_files():
    """Verify configuration files exist."""
    print("\nVerifying configuration files...")

    required_files = [
        "pyproject.toml",
        "setup.py",
        "requirements.txt",
        "requirements-dev.txt",
        "pytest.ini",
    ]

    missing = []
    for file_path in required_files:
        if not Path(file_path).is_file():
            missing.append(file_path)

    if missing:
        print(f"  ✗ Missing files: {', '.join(missing)}")
        return False

    print("  ✓ All configuration files present")
    return True


def main():
    """Run all verification checks."""
    print("=" * 60)
    print("Skills and Subagents Framework - Setup Verification")
    print("=" * 60)

    results = []
    results.append(("Structure", verify_structure()))
    results.append(("Imports", verify_imports()))
    results.append(("Config Files", verify_config_files()))

    print("\n" + "=" * 60)
    print("SUMMARY")
    print("=" * 60)

    all_passed = True
    for name, passed in results:
        status = "✓ PASS" if passed else "✗ FAIL"
        print(f"{name:20} {status}")
        all_passed = all_passed and passed

    print("=" * 60)

    if all_passed:
        print("\n✓ All checks passed! Project setup is complete.")
        print("\nNext steps:")
        print("  1. Install dependencies: pip install -r requirements-dev.txt")
        print("  2. Run tests: pytest")
        print("  3. Start Phase 2: Foundational Infrastructure")
        return 0
    else:
        print("\n✗ Some checks failed. Please review the output above.")
        return 1


if __name__ == "__main__":
    sys.exit(main())
