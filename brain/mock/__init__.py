from .backend import MockBackend
from .world import Rect, World, flat_lab

__all__ = ["MockBackend", "World", "Rect", "flat_lab", "gazebo_flat_lab", "gazebo_lab4"]


def gazebo_flat_lab() -> World:
    """The mock world generated from the real Gazebo flat_lab sources (see lab_world.py)."""
    from .lab_world import gazebo_flat_lab as build
    return build()


def gazebo_lab4(variant: str = "lab4") -> World:
    """The mock world of the Gazebo lab4 (four rooms A-D), for the public demo (see lab_world.py); variant "lab4_vln" = + a chair."""
    from .lab_world import gazebo_lab4 as build
    return build(variant)
