"""
Implementation for a simple greeting skill.
"""

from typing import Dict, Any
import asyncio


async def execute(params: Dict[str, Any]) -> Dict[str, Any]:
    """
    Greet a user by name with optional enthusiastic mode.

    Args:
        params: Dictionary containing:
            - name (str): Name of the person to greet
            - enthusiastic (bool, optional): Add excitement (default: False)

    Returns:
        Dictionary containing:
            - greeting (str): The greeting message
            - timestamp (str): When the greeting was generated
    """
    from datetime import datetime

    name = params["name"]
    enthusiastic = params.get("enthusiastic", False)

    # Simulate some processing time
    await asyncio.sleep(0.1)

    greeting = f"Hello, {name}"
    if enthusiastic:
        greeting += "! Great to see you!"
    else:
        greeting += "."

    return {
        "greeting": greeting,
        "timestamp": datetime.now().isoformat(),
    }
