from enum import Enum, IntEnum


class MatchState(IntEnum):
    Idle = 0
    PreMatch = 1
    WaitForStartOfMatch = 2
    Match = 3
    MatchFinished = 4


class Side:
    Unknown = 0
    Blue = 1
    Yellow = 2


__all = [MatchState]
