from enum import Enum, IntEnum, Optional
from dataclasses import dataclass
import math


@dataclass
class Point:
    __slots__ = ['x', 'y']
    x: float
    y: float


@dataclass
class Pose:
    __slots__ = ['x', 'y', 'yaw']
    x: float
    y: float
    yaw: float  # in radians


@dataclass
class Path:
    points: list(Point)
    cost: float


@dataclass
class Obstacle:
    vertices: list(Point)


class PathPlanner:
    def setLimits(self, xmin: float, xmax: float, ymin: float, ymax: float) -> None:
        """Set table limits"""
        return None

    def setObstacles(self, obstacles: list(Obstacle)) -> None:
        """Set obstacles        
        
        Parameters
        ----------
        obstacles : list(Obstacle)
            A list of polygonal obstacles to avoid        
        """
        return None

    def computePath(self, start: Pose, target: Pose) -> Optional(Path):
        """Compute path between two poses
        
        Parameters
        ----------
        start : Pose
            The robot's current pose
            
        target : Pose
            The robot's target pose        
        """
        return None


class TrivialPathPlanner:
    def setLimits(self, xmin: float, xmax: float, ymin: float, ymax: float) -> None:
        return None

    def setObstacles(self, obstacles: list(Obstacle)) -> None:
        return None

    def computePath(self, start: Pose, target: Pose) -> Optional(Path):
        p1 = Point(start.x, start.y)
        p2 = Point(target.x, target.y)
        dx = p2.x - p1.x
        dy = p2.y - p1.y
        distance = math.sqrt(dx ** 2 + dy ** 2)
        return Path([p1, p2], distance)
