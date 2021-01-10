import collections.abc


def nested_dict_update(base_dict, update_dict):
    """
    Update a nested dictionary or similar mapping.
    Modifies d in place.
    https://stackoverflow.com/questions/3232943/update-value-of-a-nested-dictionary-of-varying-depth
    """
    for k, v in update_dict.items():
        if isinstance(v, collections.abc.Mapping):
            base_dict[k] = nested_dict_update(base_dict.get(k, {}), v)
        else:
            base_dict[k] = v
    return base_dict


def is_list_unique(x):
    """ all elements must be hashable """
    return len(x) == len(set(x))
