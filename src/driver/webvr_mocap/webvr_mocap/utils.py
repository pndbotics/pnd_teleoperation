from typing import Any, Union


def count_elements(data: Union[dict, list, Any]) -> int:
    """Count the total number of elements in nested data structures.

    Recursively counts elements in dictionaries and lists.

    Args:
        data: The data structure to count (dict, list, or any other type).

    Returns:
        The total count of elements. For non-container types, returns 1.
    """
    if isinstance(data, dict):
        return sum(count_elements(v) for v in data.values())
    elif isinstance(data, list):
        return sum(count_elements(item) for item in data)
    else:
        return 1


if __name__ == "__main__":
    sample_data = {
        "axes": [0.0, 1.0, -1.0],
        "buttons": [
            [1, 0],
            [0, 1],
            [
                1,
            ],
            [0, 0],
        ],
    }
    total_elements = count_elements(sample_data)
    print(f"Total elements in sample data: {count_elements(sample_data['buttons'])}")
