def enforce_cast(func):
    def wrapper(*args, **kwargs):
        # Perform casting for input parameters if necessary
        new_args = [int(arg) if isinstance(arg, (str, float)) else arg for arg in args]


        # Call the original function with the casted parameters
        result = func(*new_args, **kwargs)

        # Perform casting for the output value if necessary
        if not isinstance(result, str):
            result = str(result)

        return result

    return wrapper