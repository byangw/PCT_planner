from __future__ import annotations
import a_star
import typing
import numpy
_Shape = typing.Tuple[int, ...]

__all__ = [
    "Astar",
    "DIAGONAL",
    "EUCLIDEAN",
    "HeuristicType",
    "MANHATTAN"
]


class Astar():
    def __init__(self, h_type: HeuristicType = HeuristicType.DIAGONAL) -> None: ...
    def debug(self) -> None: ...
    def get_cost_layer(self, arg0: int) -> numpy.ndarray[numpy.float64, _Shape[m, n]]: ...
    def get_ele_layer(self, arg0: int) -> numpy.ndarray[numpy.float64, _Shape[m, n]]: ...
    def get_result_matrix(self) -> numpy.ndarray[numpy.float64, _Shape[m, n]]: ...
    def get_visited_set(self) -> numpy.ndarray[numpy.int32, _Shape[m, n]]: ...
    def init(self, arg0: float, arg1: int, arg2: float, arg3: numpy.ndarray[numpy.float64, _Shape[m, n]], arg4: numpy.ndarray[numpy.float64, _Shape[m, n]], arg5: numpy.ndarray[numpy.float64, _Shape[m, n]]) -> None: ...
    def search(self, arg0: numpy.ndarray[numpy.int32, _Shape[3, 1]], arg1: numpy.ndarray[numpy.int32, _Shape[3, 1]]) -> bool: ...
    pass
class HeuristicType():
    """
    Members:

      EUCLIDEAN

      MANHATTAN

      DIAGONAL
    """
    def __eq__(self, other: object) -> bool: ...
    def __getstate__(self) -> int: ...
    def __hash__(self) -> int: ...
    def __index__(self) -> int: ...
    def __init__(self, value: int) -> None: ...
    def __int__(self) -> int: ...
    def __ne__(self, other: object) -> bool: ...
    def __repr__(self) -> str: ...
    def __setstate__(self, state: int) -> None: ...
    @property
    def name(self) -> str:
        """
        :type: str
        """
    @property
    def value(self) -> int:
        """
        :type: int
        """
    DIAGONAL: a_star.HeuristicType # value = <HeuristicType.DIAGONAL: 2>
    EUCLIDEAN: a_star.HeuristicType # value = <HeuristicType.EUCLIDEAN: 0>
    MANHATTAN: a_star.HeuristicType # value = <HeuristicType.MANHATTAN: 1>
    __members__: dict # value = {'EUCLIDEAN': <HeuristicType.EUCLIDEAN: 0>, 'MANHATTAN': <HeuristicType.MANHATTAN: 1>, 'DIAGONAL': <HeuristicType.DIAGONAL: 2>}
    pass
DIAGONAL: a_star.HeuristicType # value = <HeuristicType.DIAGONAL: 2>
EUCLIDEAN: a_star.HeuristicType # value = <HeuristicType.EUCLIDEAN: 0>
MANHATTAN: a_star.HeuristicType # value = <HeuristicType.MANHATTAN: 1>
