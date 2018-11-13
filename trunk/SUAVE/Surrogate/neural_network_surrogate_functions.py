## @ingroup Surrogate
# neural_network_surrogate_functions.py

# Created: Nov 2018, J. Smart
# Modified:


# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------

import SUAVE
from SUAVE.Core import Data, Units
from SUAVE.Methods.Weights.Buildups.Electric_Stopped_Rotor.empty import empty

from itertools import product

import numpy as np
import csv

# ----------------------------------------------------------------------
# Build Dataset
# ----------------------------------------------------------------------

def build_dataset():
    
# ----------------------------------------------------------------------
# Vehicle Level Properties
# ----------------------------------------------------------------------

    MTOW_Space      = np.linspace(2000., 20000., 200)
    battery_Space   = np.linspace(0., 1000., 100)
    payload_Space   = np.linspace(0., 5000., 500)

# ----------------------------------------------------------------------
# Fuselage Properties
# ----------------------------------------------------------------------

    fLength_Space   = np.linspace(3., 10., 20)
    fWidth_Space    = np.linspace(2., 10., 20)
    fHeight_Space   = np.linspace(1., 3., 5)

# ----------------------------------------------------------------------
# Wing Properties
# ----------------------------------------------------------------------

    span_Space      = np.linspace(0., 20., 200)
    chord_Space     = np.linspace(0., 5., 50)
    tc_Space        = np.linspace(0.01, 0.3, 10)
    wf_Space        = np.linspace(0., 0.25, 10)

# ----------------------------------------------------------------------
# Propeller Properties
# ----------------------------------------------------------------------

    liftCount_Space     = np.linspace(2, 20, 19)
    liftBlade_Space     = np.linspace(2, 10, 8)
    thrustCount_Space   = np.linspace(1, 10, 10)
    thrustBlade_Space   = np.linspace(2, 10, 8)
    tipRadius_Space     = np.linspace(0., 5., 50)

# ----------------------------------------------------------------------
# Create Specification List
# ----------------------------------------------------------------------

    specs = product(
    MTOW_Space,
    battery_Space,
    payload_Space,
    fLength_Space,
    fWidth_Space,
    fHeight_Space,
    span_Space,
    chord_Space,
    tc_Space,
    wf_Space,
    liftCount_Space,
    liftBlade_Space,
    thrustCount_Space,
    thrustBlade_Space,
    tipRadius_Space,
    )

    storage = []

    for spec in specs:

        # --------------------------------------------------------------
        # Unpack Inputs
        # --------------------------------------------------------------

        (MTOW, batmass, payload,
         fLength, fWidth, fHeight,
         span, chord, tc, wf,
         liftCount, liftBlade, thrustCount, thrustBlade, tipRadius) = spec

        vehicle = SUAVE.Vehicle()
        vehicle.tag = "Attribute Holder"

        vehicle.mass_properties.max_takeoff = MTOW

        net = SUAVE.Components.Energy.Networks.Lift_Forward_Propulsor()
        net.number_of_engines_lift = liftCount
        net.number_of_engines_forward = thrustCount
        net.number_of_engines = liftCount+thrustCount

        payload = SUAVE.Components.Energy.Peripherals.Payload()
        payload.mass_properties.mass = 200. * Units.kg
        net.payload = payload

        bat = SUAVE.Components.Energy.Storages.Batteries.Constant_Mass.Lithium_Ion()
        bat.mass_properties.mass = batmass
        net.battery = bat

        prop_lift = SUAVE.Components.Energy.Converters.Propeller()
        prop_lift.tip_radius = tipRadius
        prop_lift.number_blades = liftBlade
        prop_lift.tag = "Forward_Prop"
        net.propeller_lift = prop_lift

        prop_fwd = SUAVE.Components.Energy.Converters.Propeller()
        prop_fwd.tip_radius = tipRadius
        prop_fwd.number_blades = thrustBlade
        prop_fwd.tag = "Thrust_Prop"
        net.propeller_forward = prop_fwd

        vehicle.append_component(net)

        wing = SUAVE.Components.Wings.Main_Wing()
        wing.tag = 'main_wing'
        wing.spans.projected = span
        wing.chords.mean_aerodynamics = chord
        wing.thickness_to_chord = tc
        wing.winglet_fraction = wf
        wing.areas.reference = span * chord
        wing.motor_spanwise_locations = np.linspace(0., 1., liftCount) * span

        vehicle.append_component(wing)

        sec_wing = SUAVE.Components.Wings.Wing()
        wing.tag =  'secondary_wing'
        vehicle.append_component(wing)

        response = empty(vehicle)

        storage.append(response)

        return storage