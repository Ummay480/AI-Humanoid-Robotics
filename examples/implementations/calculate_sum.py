"""
Implementation for a calculation skill.
"""

from typing import Dict, Any, List


async def execute(params: Dict[str, Any]) -> Dict[str, Any]:
    """
    Calculate the sum of a list of numbers.

    Args:
        params: Dictionary containing:
            - numbers (list): List of numbers to sum

    Returns:
        Dictionary containing:
            - sum (float): The total sum
            - count (int): Number of items summed
            - average (float): Average value
    """
    numbers: List[float] = params["numbers"]

    if not numbers:
        return {
            "sum": 0.0,
            "count": 0,
            "average": 0.0,
        }

    total = sum(numbers)
    count = len(numbers)
    average = total / count

    return {
        "sum": total,
        "count": count,
        "average": average,
    }
