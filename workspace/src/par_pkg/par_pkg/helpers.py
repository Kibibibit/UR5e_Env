#### A collection of useful helper functions


def is_approx_equal(a, b, epsilon) -> bool:
    """
        :param a: First value to check
        :param b: Second value to check
        :param epsilon: The maximum error
        
        This function will check if a ~= b, with an error of up to epsilon
    """
    return abs(b-a) < epsilon

def clamp(min_v, max_v, v):
    """
        :param min_v: The minimum value to clamp to
        :param max_v: the maximum value to clamp to
        :param v: the value to clamp
        
        This function will clamp v to the range [min_v, max_v] inclusive.
        If min_v is greater than max_v, they will be swapped
    """
    if (min_v > max_v):
        min_v, max_v = max_v, min_v
    return min(max_v, max(min_v, v))