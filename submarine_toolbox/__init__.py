"""Submarine Toolbox — modular submarine subsystem simulation."""

__version__ = "2.0.0"

from submarine_toolbox.core.simulation import Simulation
from submarine_toolbox.dynamics.vessel import VesselParams

__all__ = ["Simulation", "VesselParams", "__version__"]
