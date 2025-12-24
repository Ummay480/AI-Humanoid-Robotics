"""
Implementation for the echo-message skill.
"""

from typing import Dict, Any


async def execute(params: Dict[str, Any]) -> Dict[str, Any]:
    """
    Echo back a message with optional prefix.

    Args:
        params: Dictionary containing:
            - message (str): Message to echo
            - prefix (str, optional): Prefix to add (default: "Echo:")

    Returns:
        Dictionary containing:
            - echoed_message (str): The prefixed message
            - original_length (int): Length of original message
            - final_length (int): Length of final message
    """
    message = params["message"]
    prefix = params.get("prefix", "Echo:")

    echoed_message = f"{prefix} {message}"

    return {
        "echoed_message": echoed_message,
        "original_length": len(message),
        "final_length": len(echoed_message),
    }
