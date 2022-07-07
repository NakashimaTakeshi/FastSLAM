def rescale_list(list_, min_value, max_value):
    """
    Args:
        list_: list to be rescaled.
        min_value: minimum value of rescaled list.
        max_value: maximum value of rescaled list.
    Returns:
        rescaled list.
    """
    return [(x - min(list_)) / (max(list_) - min(list_)) * (max_value - min_value) + min_value for x in list_]