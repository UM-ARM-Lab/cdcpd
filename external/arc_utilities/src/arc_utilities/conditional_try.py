import traceback


def conditional_try(should_catch, function, value_on_exception=None, **kwargs):
    if should_catch:
        try:
            return function(**kwargs)
        except Exception:
            print("Caught an exception!")
            traceback.print_exc()
            print("End of caught exception.")
            return value_on_exception
    else:
        return function(**kwargs)
