from aircraft_design.components.fuselage import FlexFuselage, FuselageConfiguration
from aircraft_design.components.wings.linear_taper_wing import LinearTaperWing
from aircraft_design.components.wings.basic_tail import BasicTail
from aircraft_design.components.interior.floor import FloorConfiguration
from aircraft_design.components.interior.seating import SeatConfiguration
from aircraft_design.core.component import Component
from aircraft_design.core.base import Position

def create_large_passenger_aircraft() -> Component:
    """Create a large passenger aircraft similar to provided example"""
    
    # Create main aircraft component
    aircraft = Component("large_passenger_aircraft")
    
    # 1. Create Fuselage
    # First, set up floor configurations
    floor_configs = [
        # Cargo deck
        FloorConfiguration(
            ceiling_height=7.0,
            floor_thickness=0.2,
            wall_thickness=0.15,
            is_cargo=True
        ),
        # Main passenger deck
        FloorConfiguration(
            ceiling_height=7.0,
            floor_thickness=0.2,
            wall_thickness=0.15,
        ),
        # Upper passenger deck
        FloorConfiguration(
            ceiling_height=7.0,
            floor_thickness=0.2,
            wall_thickness=0.15,
        ),
        # First class deck
        FloorConfiguration(
            ceiling_height=7.0,
            floor_thickness=0.2,
            wall_thickness=0.15,
        ),
    ]
    
    # Create fuselage configuration
    fuselage_config = FuselageConfiguration(
        total_length=250,  # Estimated from original config
        max_width=30,      # Based on seat width * seats per row
        nose_length=20,    # From cockpit_length
        tail_length=40,    # Estimated
        floor_configs=floor_configs,
        nose_taper_ratio=0.3,
        tail_taper_ratio=0.2
    )
    
    # Create and add fuselage
    fuselage = FlexFuselage("fuselage", fuselage_config)
    aircraft.add_child(fuselage)
    
    # 2. Create Wing
    # Original data: wingspan=311, sweep=37.5, 
    # wing_chord_data=[(0, 65), (30, 45), (100, 15)]
    main_wing = LinearTaperWing(
        name="main_wing",
        half_span=311/2,  # Half of total wingspan
        root_chord=65,
        tip_chord=15
    )
    
    # Position wing
    main_wing.reference_frame.position = Position(
        x=100,  # Positioned at 40% of fuselage length
        y=0,    # Centered
        z=7     # At first passenger deck height
    )
    aircraft.add_child(main_wing)
    
    # 3. Create Vertical Tail
    # Original data: v_span=20, v_sweep=35, v_chord_data=[(0, 60), (100, 40)]
    vertical_tail = BasicTail(
        name="vertical_tail",
        half_span=20,        # Full height
        root_chord=60,
        tip_chord=40,
        tail_arm=40,         # Distance from wing
        tail_type="vertical"
    )
    
    # Position vertical tail
    vertical_tail.reference_frame.position = Position(
        x=210,  # Near rear of fuselage
        y=0,    # Centered
        z=28    # Top of fuselage
    )
    aircraft.add_child(vertical_tail)
    
    # 4. Create Horizontal Tail
    # Original data: h_span=100, h_sweep=35, h_chord_data=[(0, 40), (100, 10)]
    horizontal_tail = BasicTail(
        name="horizontal_tail",
        half_span=50,        # Half of total span
        root_chord=40,
        tip_chord=10,
        tail_arm=40,         # Distance from wing
        tail_type="horizontal"
    )
    
    # Position horizontal tail at top of vertical tail
    horizontal_tail.reference_frame.position = Position(
        x=210,  # Same as vertical tail
        y=0,    # Centered
        z=48    # Top of vertical tail
    )
    aircraft.add_child(horizontal_tail)
    
    return aircraft

def main():
    # Create aircraft
    aircraft = create_large_passenger_aircraft()
    
    # Run some basic analyses
    wing = aircraft.children[1]  # Main wing
    ar_results = wing.run_analysis("wing_aspect_ratio")
    print(f"Wing Aspect Ratio: {ar_results.data['aspect_ratio']:.2f}")
    
    fuselage = aircraft.children[0]  # Fuselage
    wetted_area = fuselage.wetted_area
    print(f"Fuselage Wetted Area: {wetted_area:.2f} m²")
    
    print(f"Cargo Volume: {fuselage.get_cargo_volume():.2f} m³")
    print(f"Passenger Volume: {fuselage.get_passenger_volume():.2f} m³")

if __name__ == "__main__":
    main()

