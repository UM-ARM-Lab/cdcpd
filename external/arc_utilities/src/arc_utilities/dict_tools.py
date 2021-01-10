from typing import Dict


def dict_round(d: Dict, places: int = 2):
    return {k: round(v, places) for k, v in d.items()}
