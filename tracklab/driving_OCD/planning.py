"""Reuse the geometry-only speed schedule for the native OCD driving path.

This planner supplies tracking references; it neither substitutes for the native
vehicle equations nor modifies the track geometry.
"""
from ..driving.planning import CurvatureSpeedPlan

__all__ = ["CurvatureSpeedPlan"]
