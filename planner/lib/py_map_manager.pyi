from __future__ import annotations
import py_map_manager
import typing

__all__ = [
    "DenseElevationMap"
]


class DenseElevationMap():
    def __init__(self) -> None: ...
    def set_debug(self, arg0: bool) -> None: ...
    def update_layer(self, arg0: int, arg1: float, arg2: float) -> int: ...
    def update_layer_safe(self, arg0: int, arg1: float, arg2: float, arg3: float) -> int: ...
    pass
