from submarine_toolbox.subsystems.atmosphere import Atmosphere
from submarine_toolbox.subsystems.autopilot import Autopilot
from submarine_toolbox.subsystems.ballast import Ballast
from submarine_toolbox.subsystems.comms import Comms
from submarine_toolbox.subsystems.contacts import ContactManager
from submarine_toolbox.subsystems.control_surfaces import ControlSurfaces
from submarine_toolbox.subsystems.damage import DamageControl
from submarine_toolbox.subsystems.endurance import Endurance
from submarine_toolbox.subsystems.environment import Environment
from submarine_toolbox.subsystems.hydraulics import Hydraulics
from submarine_toolbox.subsystems.masts import Masts, RadarESM
from submarine_toolbox.subsystems.navigation import Navigation
from submarine_toolbox.subsystems.power import Power
from submarine_toolbox.subsystems.propulsion import Propulsion
from submarine_toolbox.subsystems.signature import Signature
from submarine_toolbox.subsystems.sonar import ActiveSonar, PassiveSonar
from submarine_toolbox.subsystems.tma import TMA

__all__ = ["ActiveSonar", "Atmosphere", "Autopilot", "Ballast", "Comms",
           "ContactManager", "ControlSurfaces", "DamageControl",
           "Endurance", "Environment", "Hydraulics", "Masts",
           "Navigation", "PassiveSonar", "Power", "Propulsion",
           "RadarESM", "Signature", "TMA", "build_default_boat"]

def build_default_boat(sim):
    """Attach the default subsystem stack in execution order."""
    sim.add(Environment)
    sim.add(ContactManager)
    sim.add(Autopilot)
    sim.add(DamageControl)
    sim.add(Hydraulics)
    sim.add(Ballast)
    sim.add(ControlSurfaces)
    sim.add(Propulsion)
    sim.add(Masts)
    sim.add(Atmosphere)
    sim.add(RadarESM)
    sim.add(Power)
    sim.add(Signature)
    sim.add(PassiveSonar)
    sim.add(ActiveSonar)
    sim.add(Navigation)
    sim.add(TMA)
    sim.add(Comms)
    sim.add(Endurance)
    return sim
