#!/usr/bin/env python3
"""
Test script for system metrics functionality
"""
import sys
import os

# Add the backend app directory to the path to import our modules
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '1-docusaurus-textbook', 'backend'))

def test_system_metrics():
    """Test the system metrics functionality"""
    try:
        from app.utils.system_metrics import get_system_metrics, get_detailed_health_status
        
        print("Testing system metrics functionality...")
        
        # Test basic system metrics
        metrics = get_system_metrics()
        print(f"✓ System metrics collected: {len(metrics)} keys")
        
        # Verify required keys exist
        required_keys = ['cpu', 'memory', 'disk', 'uptime', 'timestamp']
        for key in required_keys:
            assert key in metrics, f"Missing key: {key}"
        print("✓ All required keys present in system metrics")
        
        # Test detailed health status
        health_status = get_detailed_health_status()
        print(f"✓ Detailed health status collected: {len(health_status)} keys")
        
        # Verify required keys exist in health status
        required_health_keys = ['status', 'timestamp', 'uptime', 'system_metrics', 'details']
        for key in required_health_keys:
            assert key in health_status, f"Missing key in health status: {key}"
        print("✓ All required keys present in health status")
        
        print("\nSystem Metrics Sample:")
        print(f"  CPU Usage: {health_status['system_metrics']['cpu_percent']}%")
        print(f"  Memory Usage: {health_status['system_metrics']['memory_used_percent']}%")
        print(f"  Disk Usage: {health_status['system_metrics']['disk_used_percent']}%")
        print(f"  Uptime: {health_status['uptime']['hours']}h {health_status['uptime']['minutes'] % 60}m")
        
        print("\n✓ All tests passed! System metrics functionality is working correctly.")
        
    except ImportError as e:
        print(f"✗ Import error: {e}")
        return False
    except AssertionError as e:
        print(f"✗ Assertion error: {e}")
        return False
    except Exception as e:
        print(f"✗ Unexpected error: {e}")
        return False
        
    return True

if __name__ == "__main__":
    success = test_system_metrics()
    if not success:
        sys.exit(1)