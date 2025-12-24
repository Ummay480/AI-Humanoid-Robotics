"""
System Metrics Utility Module
Provides system-level information for health checks and monitoring
"""
import psutil
import os
import time
from datetime import datetime
from typing import Dict, Optional


# Store application start time
_START_TIME = time.time()


def get_system_metrics() -> Dict:
    """
    Collect comprehensive system metrics
    
    Returns:
        Dict containing system metrics including CPU, memory, disk, and uptime
    """
    # CPU usage percentage
    cpu_percent = psutil.cpu_percent(interval=1)
    
    # Memory usage
    memory = psutil.virtual_memory()
    memory_info = {
        "total_gb": round(memory.total / (1024**3), 2),
        "available_gb": round(memory.available / (1024**3), 2),
        "used_gb": round(memory.used / (1024**3), 2),
        "percentage": memory.percent
    }
    
    # Disk usage for the root directory
    disk_usage = psutil.disk_usage('/')
    disk_info = {
        "total_gb": round(disk_usage.total / (1024**3), 2),
        "used_gb": round(disk_usage.used / (1024**3), 2),
        "free_gb": round(disk_usage.free / (1024**3), 2),
        "percentage": round((disk_usage.used / disk_usage.total) * 100, 2)
    }
    
    # Calculate uptime
    uptime_seconds = time.time() - _START_TIME
    uptime_info = {
        "seconds": int(uptime_seconds),
        "minutes": int(uptime_seconds / 60),
        "hours": int(uptime_seconds / 3600),
        "days": int(uptime_seconds / 86400)
    }
    
    return {
        "cpu": {
            "percent": cpu_percent
        },
        "memory": memory_info,
        "disk": disk_info,
        "uptime": uptime_info,
        "timestamp": datetime.utcnow().isoformat()
    }


def get_detailed_health_status() -> Dict:
    """
    Get detailed health status including system metrics
    
    Returns:
        Dict containing detailed health status with system information
    """
    system_metrics = get_system_metrics()
    
    return {
        "status": "healthy",
        "timestamp": datetime.utcnow().isoformat(),
        "uptime": system_metrics["uptime"],
        "system_metrics": {
            "cpu_percent": system_metrics["cpu"]["percent"],
            "memory_used_percent": system_metrics["memory"]["percentage"],
            "disk_used_percent": system_metrics["disk"]["percentage"],
        },
        "details": {
            "memory_gb": {
                "total": system_metrics["memory"]["total_gb"],
                "used": system_metrics["memory"]["used_gb"],
                "available": system_metrics["memory"]["available_gb"]
            },
            "disk_gb": {
                "total": system_metrics["disk"]["total_gb"],
                "used": system_metrics["disk"]["used_gb"],
                "free": system_metrics["disk"]["free_gb"]
            }
        }
    }