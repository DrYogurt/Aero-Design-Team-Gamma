from typing import List, Dict
import numpy as np
from aircraft_design.components.interior.blended_wing_cabin import (
    EconomySection, ExitRow, EconomyBlock, SingleFloorCabin
)
from aircraft_design.analysis.mass_analysis import MassFeature, MassAnalysis
from aircraft_design.components.interior.service import Galley, Bathroom

class MassEconomySection(EconomySection):
    """Economy section with mass properties"""
    
    # Typical mass values in kg
    SEAT_MASS = 12  # Average economy seat mass
    
    def __init__(self, name: str, seat_groups: List[int], num_rows: int = 1):
        super().__init__(name, seat_groups, num_rows)
        
        # Calculate total section mass based on seats
        total_seats = sum(seat_groups) * num_rows
        section_mass = total_seats * self.SEAT_MASS
        
        # Add mass feature and analysis
        self.add_feature(MassFeature(mass=section_mass))
        self.add_analysis(MassAnalysis())

class MassExitRow(ExitRow):
    """Exit row with mass properties"""
    
    # Typical mass values in kg
    EXIT_DOOR_MASS = 50  # Emergency exit door mass
    
    def __init__(self, name: str):
        super().__init__(name)
        self.add_feature(MassFeature(mass=self.EXIT_DOOR_MASS))
        self.add_analysis(MassAnalysis())

class MassEconomyBlock(EconomyBlock):
    """Economy block with mass properties"""
    
    def __init__(self, name: str, seat_groups: List[int], rows_per_section: List[int], 
                 seats_per_exit: int = 60):
        super().__init__(name, seat_groups, rows_per_section, seats_per_exit)
        
        # Add mass feature for structural mass (frames, floor panels, etc)
        structural_mass = self.total_length * self.total_width * 10  # 10 kg/m² as example
        self.add_feature(MassFeature(mass=structural_mass))
        self.add_analysis(MassAnalysis())

class MassGalley(Galley):
    """Galley with mass properties"""
    
    # Typical mass values in kg
    BASE_MASS = 200  # Base galley structure
    MASS_PER_VOLUME = 50  # Additional mass per cubic meter
    
    def __init__(self, name: str, width: float, depth: float, height: float):
        super().__init__(name, width, depth, height)
        volume = width * depth * height
        total_mass = self.BASE_MASS + (volume * self.MASS_PER_VOLUME)
        self.add_feature(MassFeature(mass=total_mass))
        self.add_analysis(MassAnalysis())

class MassBathroom(Bathroom):
    """Bathroom with mass properties"""
    
    # Typical mass values in kg
    BATHROOM_MASS = 150  # Standard lavatory mass
    
    def __init__(self, name: str, width: float, depth: float):
        super().__init__(name, width, depth)
        self.add_feature(MassFeature(mass=self.BATHROOM_MASS))
        self.add_analysis(MassAnalysis())

class MassSingleFloorCabin(SingleFloorCabin):
    """Single floor cabin with mass properties"""
    
    def __init__(self, name: str):
        super().__init__(name)
        # Add mass feature for basic cabin structure
        self.add_feature(MassFeature(mass=0))  # Base mass will be updated when components are added
        self.add_analysis(MassAnalysis())

def main():
    """Create and analyze a cabin layout with mass properties"""
    # Create the single floor cabin
    cabin = MassSingleFloorCabin("main_cabin")
    
    # Create front economy block
    front_economy = MassEconomyBlock(
        name="front_economy",
        seat_groups=[3, 4, 3],
        rows_per_section=[4, 4],
        seats_per_exit=60
    )
    
    # Create middle economy block
    middle_economy = MassEconomyBlock(
        name="middle_economy",
        seat_groups=[3, 6, 3],
        rows_per_section=[4, 5, 4]
    )
    
    # Create rear economy block
    rear_economy = MassEconomyBlock(
        name="rear_economy",
        seat_groups=[3, 6, 3],
        rows_per_section=[6, 8],
        seats_per_exit=60
    )
    
    # Create galleys and bathrooms
    GALLEY_SPACING = 2.0
    mid_galley = MassGalley("mid_galley", width=10.0, depth=4.0, height=7.0)
    rear_galley = MassGalley("rear_galley", width=15.0, depth=4.0, height=7.0)
    
    bathroom_width = 3.0
    bathroom_depth = 3.0
    mid_bathroom = MassBathroom("mid_bathroom", width=bathroom_width, depth=bathroom_depth)
    back_bathroom = MassBathroom("back_bathroom", width=bathroom_width, depth=bathroom_depth)
    
    # Add components to cabin in sequence
    current_x = 0.0
    
    # Front economy block
    cabin.add_component(front_economy, current_x)
    current_x += front_economy.total_length + GALLEY_SPACING
    
    # Mid galley and bathroom
    cabin.add_component(mid_galley, current_x)
    cabin.add_component(mid_bathroom, current_x)
    current_x += mid_galley.geometry.parameters['depth'] + GALLEY_SPACING
    
    # Middle economy block
    cabin.add_component(middle_economy, current_x)
    current_x += middle_economy.total_length + GALLEY_SPACING
    
    # Rear economy block
    cabin.add_component(rear_economy, current_x)
    current_x += rear_economy.total_length + GALLEY_SPACING
    
    # Rear galley and bathroom
    cabin.add_component(rear_galley, current_x)
    cabin.add_component(back_bathroom, current_x)
    
    # Run mass analysis and print results
    mass_results = cabin.run_analysis("mass_analysis")
    
    print("\nMass Analysis Results:")
    print(f"Total Cabin Mass: {mass_results['total_mass']:.1f} kg")
    
    # Print individual component masses
    for component in cabin.components:
        component_mass = component.run_analysis("mass_analysis")
        print(f"{component.name} Mass: {component_mass['total_mass']:.1f} kg")

if __name__ == "__main__":
    main() 