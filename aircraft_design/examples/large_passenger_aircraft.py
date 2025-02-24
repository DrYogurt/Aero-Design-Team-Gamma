from typing import List, Dict, Any, Optional, Tuple, Union
from aircraft_design.core.base import Component, Position
from aircraft_design.components.aerodynamics.basic_aero import AerodynamicComponent
from aircraft_design.components.fuselage.fuselage_geometry import FuselageGeometry, CrossSection
from aircraft_design.components.aerodynamics.wing_geometry import WaypointWingGeometry, TailGeometry, SimpleSweptWing
from aircraft_design.components.interior.floor import Floor
from aircraft_design.components.interior.seating import SeatingSection
from aircraft_design.components.interior.service import Galley
from aircraft_design.core.plotting import Object3D, plot_3d_object, plot_orthographic_views
import numpy as np
import matplotlib.pyplot as plt
from aircraft_design.components.propulsion.engine import Engine, EngineGeometry

class Aircraft(Component):
    """Top level aircraft class that manages all components and configurations"""
    def __init__(self, name: str):
        super().__init__(name)
        self.configuration: Dict[str, Any] = {}
        # Dictionary to store components by type
        self._components: Dict[str, Union[Component, List[Component]]] = {}
        
    def _register_component(self, component: Component, type_name: str) -> None:
        """Register a component in the type-based lookup dictionary
        
        If there's already a component of this type:
        - If it's a single component, convert to list
        - If it's a list, append to it
        - If it's None, set it directly
        """
        if type_name in self._components:
            if isinstance(self._components[type_name], list):
                self._components[type_name].append(component)
            else:
                # Convert to list with both components
                self._components[type_name] = [self._components[type_name], component]
        else:
            self._components[type_name] = component
    
    def __getattr__(self, name: str) -> Union[Component, List[Component]]:
        """Allow access to components by type name as attributes"""
        if name in self._components:
            return self._components[name]
        raise AttributeError(f"'{self.__class__.__name__}' has no attribute '{name}'")

    def add_fuselage(self, 
                     total_passengers: int,
                     seat_configs: List[Dict],
                     galleys: List[Dict],
                     cockpit_length: float,
                     cargo_per_passenger: float,
                     nose_fineness: float = 2.0,
                     tail_length: float = 40.0,
                     tail_grade: float = 0.3) -> None:
        """Configure and add fuselage based on passenger requirements
        
        Args:
            total_passengers: Total number of passengers
            seat_configs: List of seat configuration dictionaries
            galleys: List of galley configuration dictionaries
            cockpit_length: Length of cockpit section in feet
            cargo_per_passenger: Required cargo volume per passenger in cubic feet
            nose_fineness: Length/diameter ratio for nose section (default=2.0)
            tail_length: Length of tail section in feet (default=40.0)
            tail_grade: Controls tail taper curve (0=sharp taper, 1=linear, default=0.3)
        """
        from aircraft_design.components.interior.floor import Floor
        from aircraft_design.components.interior.seating import SeatingSection
        from aircraft_design.components.interior.service import Galley, Bathroom
        from aircraft_design.components.fuselage.fuselage_geometry import CrossSection, CrossSectionShape
        
        # Create fuselage geometry
        fuselage = FuselageGeometry()
        # Set fineness ratios first
        fuselage.parameters.update({
            'nose_fineness': nose_fineness,
            'tail_length': tail_length,
            'tail_grade': tail_grade
        })
        
        # Calculate basic dimensions
        max_width = 0.0
        max_height = 0.0
        total_length = cockpit_length
        
        # Calculate required width and height from seat configs
        for config in seat_configs:
            if config.get('cargo', False):
                continue
                
            # Calculate width needed for seats and aisle
            total_seats = sum(config['seat_distribution'])
            width = (total_seats * config['seat_width'] + 
                    (len(config['seat_distribution']) - 1) * config['aisle_width'])
            max_width = max(max_width, width)
            
            # Calculate height including headroom
            max_height += config['ceiling_height']
            
        # Add 20% margin for walls and systems
        max_width *= 1.2
        max_height *= 1.2
        
        # Calculate cabin length based on total passengers and seat configs
        rows_per_config = []
        seats_per_row = sum(
            sum(config['seat_distribution'])
            for config in seat_configs
            if not config.get('cargo', False)
        )
        print(f"seats per row: {seats_per_row}")
        rows_per_config = -(-total_passengers // seats_per_row)
        total_length = rows_per_config * max(config['seat_depth'] for config in seat_configs) 
        
        # Add length for galleys
        for galley in galleys:
            total_length += galley['length']
            
        # Add cargo section length based on cargo requirements
        cargo_area = sum(config['ceiling_height'] * config['seat_width']*sum(config['seat_distribution']) for config in seat_configs if config.get('cargo', False))
        available_cargo_volume = total_length * cargo_area
        # if there isn't enough space for the cargo, add the necessary cargo length to the total length
        if available_cargo_volume < total_passengers * cargo_per_passenger:
            total_length += (total_passengers * cargo_per_passenger - available_cargo_volume) / cargo_area
        
        # Create cross-sections
        # Nose section - create more sections for smoother transition
        nose_length = cockpit_length
        nose_sections = [
            (0.0, 0.1, 0.1),  # Nose tip
            (nose_length * 0.1, max_width * 0.2, max_height * 0.2),  # 10% of nose
            (nose_length * 0.3, max_width * 0.5, max_height * 0.5),  # 30% of nose
            (nose_length * 0.6, max_width * 0.8, max_height * 0.8),  # 60% of nose
            (nose_length, max_width, max_height)  # End of nose
        ]
        for station, width, height in nose_sections:
            fuselage.add_section(CrossSection(station, width, height, CrossSectionShape.SUPER_ELLIPSE))
        
        # Constant section
        fuselage.add_section(CrossSection(nose_length, max_width, max_height, CrossSectionShape.SUPER_ELLIPSE))
        
        # Tail section - create more sections for smoother transition
        tail_start = nose_length + total_length
        tail_sections = []
        
        # Create more sections for smoother transition
        num_tail_sections = 8
        for i in range(num_tail_sections + 1):
            station_frac = i / num_tail_sections  # 0 to 1
            station = tail_start + station_frac * tail_length
            
            # Use power function for taper based on grade
            # grade=0 gives sharp early taper, grade=1 gives linear taper
            taper_factor = (1 - station_frac) ** (1 / max(tail_grade, 0.01))
            
            width = max_width * taper_factor
            height = max_height * taper_factor
            
            # Ensure minimum size at tail tip
            if i == num_tail_sections:  # Last section
                width = max(width, max_width * 0.05)
                height = max(height, max_height * 0.05)
            
            tail_sections.append((station, width, height))
        
        for station, width, height in tail_sections:
            fuselage.add_section(CrossSection(station, width, height, CrossSectionShape.SUPER_ELLIPSE))

        # Update fuselage parameters
        fuselage.parameters.update({
            'length': total_length + nose_length + tail_length,
            'max_width': max_width,
            'max_height': max_height,
        })
        
        # Create fuselage component
        fuselage_component = AerodynamicComponent("fuselage")
        fuselage_component.geometry = fuselage
        
        # Create passenger floors
        passenger_floors = []
        for floor_idx, config in enumerate(seat_configs):
            if config.get('cargo', False):
                continue
            
            floor = Floor(f"passenger_floor_{floor_idx}", "passenger")
            
            # Add main seating section

            
            section = SeatingSection(f"main_section_{floor_idx}")
            for group_size in config['seat_distribution']:
                section.add_seat_group(group_size, config.get('seat_class', 'economy'))
            
            floor.add_section(section)
            passenger_floors.append(floor)
        
        # Create cargo floor
        cargo_floor = Floor("cargo_floor", "cargo")
        cargo_section = SeatingSection("cargo_section", "cargo")
        cargo_floor.add_section(cargo_section)
        
        # Add service areas
        for galley_config in galleys:
            floor_idx = galley_config['floor']
            for i in range(galley_config['number']):
                galley = Galley(f"galley_{floor_idx}_{i}")
                passenger_floors[floor_idx].sections[0].add_service_area(galley)
        
        # Add floors as children of fuselage
        for floor in passenger_floors + [cargo_floor]:
            fuselage_component.add_child(floor)
        
        # Register the fuselage component
        self._register_component(fuselage_component, 'fuselage')
        self.add_child(fuselage_component)

    def add_wing(self,
                 chord_points: List[Tuple[float, float]],
                 thicknesses: List[float],
                 half_span: float,
                 sweep: float = 0.0,
                 dihedral: float = 0.0,
                 position: Optional[Position] = None) -> None:
        """Add main wing with given parameters, using chord points and thicknesses to generate waypoints"""
        if len(chord_points) != len(thicknesses):
            raise ValueError("chord_points and thicknesses must have the same length")
        
        # Convert sweep angle to radians
        sweep_rad = np.radians(sweep)
        
        # Calculate the total span from the chord points
        total_span = half_span * 2
        
        # Generate waypoints from chord points and thicknesses, adjusting for sweep
        waypoints = []
        for (chord, span_fraction), thickness in zip(chord_points, thicknesses):
            # Calculate normalized span fraction
            
            waypoints.append({
                'span_fraction': span_fraction,
                'chord': chord,
                'thickness': thickness
            })
        
        wing = WaypointWingGeometry(waypoints)
        wing_component = AerodynamicComponent("main_wing")
        wing_component.geometry = wing
        wing_component.geometry.position = position if position else Position(0, 0, 0)
        
        # Update parameters which will automatically create waypoints
        wing.update_parameters({
            'span': total_span,
            'sweep': sweep,
            'dihedral': dihedral,
            'root_chord': chord_points[0][0],
            'tip_chord': chord_points[-1][0]
        })
        
        # Register the wing component
        self._register_component(wing_component, 'wing')
        self.add_child(wing_component)

    def add_tail(self,
                 half_span: float,
                 root_chord: float,
                 tip_chord: float,
                 sweep: float = 0.0,
                 tail_type: str = 'vertical',
                 position: Optional[Position] = None,
                 cant_angle: float = 0.0) -> None:
        """Add vertical or horizontal tail
        
        Args:
            half_span: Half-span for horizontal tail or height for vertical tail (ft)
            root_chord: Root chord length (ft)
            tip_chord: Tip chord length (ft)
            sweep: Quarter-chord sweep angle (degrees)
            tail_type: Either 'vertical' or 'horizontal'
            position: Position of the tail root
            cant_angle: Angle from vertical in degrees (0 = vertical, positive = outward cant)
        """
        # Set parameters based on tail type
        if tail_type == 'vertical':
            tail = TailGeometry()
            tail.position = position if position else Position(0, 0, 0)
            tail.parameters.update({
                'height': half_span,  # Convert to full height
                'root_chord': root_chord,
                'tip_chord': tip_chord,
                'sweep': sweep,
                'cant_angle': cant_angle
            })
            name = "vertical_tail"
        else:  # horizontal tail should have a simple swept wing geometry
            tail = SimpleSweptWing(root_chord, tip_chord, half_span * 2, sweep)
            tail.position = position if position else Position(0, 0, 0)
            name = "horizontal_tail"
        
        tail_component = AerodynamicComponent(name)
        tail_component.geometry = tail
        
        if position:
            tail_component.geometry.position = position
            
        # Register the tail component
        component_type = 'vertical_tail' if tail_type == 'vertical' else 'horizontal_tail'
        self._register_component(tail_component, component_type)
        
        if tail_type == 'vertical':
            self.add_child(tail_component)
        else:  # horizontal
            vtail = next((c for c in self.children if c.name == "vertical_tail"), None)
            if not vtail:
                raise ValueError("Cannot add horizontal tail: Vertical tail must be added first")
            vtail.add_child(tail_component)

    def add_engines(self, engine_configs: List[Dict]) -> None:
        """Add engines with given configurations"""
        from aircraft_design.components.propulsion.engine import Engine
        
        # Get the wing component
        wing = next((c for c in self.children if c.name == "main_wing"), None)
        if not wing:
            raise ValueError("Cannot add engines: Wing must be added first")
        
        # Get wing position
        wing_pos = wing.geometry.position
        
        for idx, config in enumerate(engine_configs):
            for pos_idx, (x, y, z) in enumerate(config['positions']):
                # Create primary side engine
                engine_instance = Engine(f"engine_{idx}_pos_{pos_idx}")
                engine_instance.geometry = EngineGeometry()
                engine_instance.geometry.parameters.update({
                    'radius': config['radius'],
                    'length': config['length']
                })
                engine_instance.geometry.position = Position(
                    x + wing_pos.x,
                    y + wing_pos.y,
                    z + wing_pos.z
                )
                self._register_component(engine_instance, 'engines')
                wing.add_child(engine_instance)
                
                # Create secondary side engine
                engine_instance = Engine(f"engine_{idx}_pos_{pos_idx + 1}")
                engine_instance.geometry = EngineGeometry()
                engine_instance.geometry.parameters.update({
                    'radius': config['radius'],
                    'length': config['length']
                })
                engine_instance.geometry.position = Position(
                    x + wing_pos.x,
                    -y + wing_pos.y,
                    z + wing_pos.z
                )
                self._register_component(engine_instance, 'engines')
                wing.add_child(engine_instance)

    def analyze_performance(self) -> Dict[str, Dict[str, float]]:
        """Run all available analyses on the aircraft and its components"""
        results = {
            'aircraft': {},
            'wing': {},
            'fuselage': {},
            'tail': {},
            'engines': {}
        }
        
        # Get main components
        wing = next((c for c in self.children if c.name == "main_wing"), None)
        fuselage = next((c for c in self.children if c.name == "fuselage"), None)
        vtail = next((c for c in self.children if c.name == "vertical_tail"), None)
        htail = next((c for c in self.children if c.name == "horizontal_tail"), None)
        engines = [c for c in self.children if c.name.startswith("engine_")]
        
        # Wing analysis
        if wing:
            # Basic lift analysis
            lift_analysis = wing.analyses.get('basic_lift')
            if lift_analysis:
                lift_analysis.parameters.update({
                    'alpha': 5.0,  # cruise angle of attack
                    'mach': 0.85,  # cruise Mach number
                    'reynolds': 1e7  # approximate Reynolds number
                })
                try:
                    results['wing'].update(wing.run_analysis('basic_lift'))
                except Exception as e:
                    print(f"Warning: Wing lift analysis failed: {e}")
                    results['wing']['CL'] = 0.0
                    results['wing']['CL_alpha'] = 0.0
            
            # Drag analysis
            drag_analysis = wing.analyses.get('parasitic_drag')
            if drag_analysis:
                drag_analysis.parameters.update({
                    'mach': 0.85,
                    'reynolds': 1e7
                })
                try:
                    results['wing'].update(wing.run_analysis('parasitic_drag'))
                except Exception as e:
                    print(f"Warning: Wing drag analysis failed: {e}")
                    results['wing']['CD'] = 0.0
        
        # Fuselage analysis
        if fuselage:
            results['fuselage'].update({
                'wetted_area': getattr(fuselage.geometry, 'calculate_wetted_area', lambda: 0.0)(),
            })
        
        # Tail analyses
        if vtail:
            vtail_lift = vtail.analyses.get('basic_lift')
            if vtail_lift:
                vtail_lift.parameters.update({
                    'alpha': 0.0,  # nominal vertical tail angle
                    'mach': 0.85
                })
                try:
                    results['tail']['vertical'] = vtail.run_analysis('basic_lift')
                except Exception as e:
                    print(f"Warning: Vertical tail analysis failed: {e}")
                    results['tail']['vertical'] = {'CL': 0.0, 'CD': 0.0}
        
        if htail:
            htail_lift = htail.analyses.get('basic_lift')
            if htail_lift:
                htail_lift.parameters.update({
                    'alpha': 2.0,  # trim angle
                    'mach': 0.85
                })
                try:
                    results['tail']['horizontal'] = htail.run_analysis('basic_lift')
                except Exception as e:
                    print(f"Warning: Horizontal tail analysis failed: {e}")
                    results['tail']['horizontal'] = {'CL': 0.0, 'CD': 0.0}
        
        # Engine analysis
        for idx, engine in enumerate(engines):
            engine_analysis = engine.analyses.get('thrust')
            if engine_analysis:
                try:
                    results['engines'][f'engine_{idx}'] = engine.run_analysis('thrust')
                except Exception as e:
                    print(f"Warning: Engine {idx} analysis failed: {e}")
                    results['engines'][f'engine_{idx}'] = {'thrust': 0.0}
        
        # Overall aircraft performance
        if wing and fuselage:
            # Calculate total drag
            total_drag = sum(
                comp.get('CD', 0) 
                for comp in [results['wing'], results['tail'].get('vertical', {}), 
                           results['tail'].get('horizontal', {})]
            )
            
            # Calculate L/D ratio
            if total_drag > 0:
                results['aircraft']['L_D_ratio'] = results['wing'].get('CL', 0) / total_drag
            else:
                results['aircraft']['L_D_ratio'] = 0.0
            
            # Calculate range using Breguet range equation
            if 'L_D_ratio' in results['aircraft']:
                W_fuel = self.configuration.get('fuel_weight', 0)
                W_total = sum(getattr(c, 'weight', 0) for c in self.children)
                TSFC = self.configuration.get('TSFC', 0.5)  # Default TSFC
                
                if W_fuel > 0 and W_total > W_fuel and TSFC > 0:
                    results['aircraft']['range'] = (
                        2/TSFC * 
                        results['aircraft']['L_D_ratio'] * 
                        np.log(W_total/(W_total - W_fuel))
                    )
                else:
                    results['aircraft']['range'] = 0.0
        
        return results


    def plot_3d(self, figsize: Tuple[int, int] = (12, 8)) -> Tuple[plt.Figure, plt.Axes]:
        """Create a 3D plot of the complete aircraft"""
        fig = plt.figure(figsize=figsize)
        ax = fig.add_subplot(111, projection='3d')
        
        # Define colors for each component type
        colors = {
            'fuselage': 'gray',
            'main_wing': 'blue',
            'vertical_tail': 'red',
            'horizontal_tail': 'green',
            'engine': 'purple'
        }
        
        # Plot the complete aircraft with colors
        aircraft_obj = self.plot(color='gray', colors_dict=colors)
        
        # Plot each shape with its color
        if aircraft_obj and aircraft_obj.shapes:
            for shape in aircraft_obj.shapes:
                color = shape.metadata.get('color', 'gray')
                plot_3d_object(ax, Object3D([shape]), color=color, alpha=0.6)
        
        # Set equal aspect ratio
        max_range = 0
        for axis in (ax.xaxis, ax.yaxis, ax.zaxis):
            max_range = max(max_range, np.ptp(axis.get_data_interval()))
        
        center = np.array([
            np.mean(ax.get_xlim()),
            np.mean(ax.get_ylim()),
            np.mean(ax.get_zlim())
        ])
        
        for direction in (-1, 1):
            for point in np.diag(direction * max_range * np.array([1, 1, 1])):
                ax.plot([point[0]], [point[1]], [point[2]], 'w')
        
        # Set labels and title
        ax.set_xlabel('X (ft)')
        ax.set_ylabel('Y (ft)')
        ax.set_zlabel('Z (ft)')
        ax.set_title('3D Aircraft View')
        
        # Add a grid
        ax.grid(True)
        
        return fig, ax

    def plot_views(self, figsize: Tuple[int, int] = (15, 10)) -> Tuple[plt.Figure, Tuple[plt.Axes, plt.Axes, plt.Axes]]:
        """Create three-view plot of the aircraft"""
        # Define colors for each component type
        colors = {
            'fuselage': 'gray',
            'main_wing': 'blue',
            'vertical_tail': 'red',
            'horizontal_tail': 'green',
            'engine': 'purple'
        }
        
        # Get the combined 3D object with all components and their colors
        aircraft_obj = self.plot(color='gray', colors_dict=colors)
        
        # Use the plotting utility to create orthographic views
        return plot_orthographic_views(aircraft_obj, figsize=figsize)

def main():
    """Create example large passenger aircraft"""
    # Create seat configurations matching original example
    seat_configs = [
        {
            'seat_width': 14,
            'seat_depth': 1,    
            'seat_height': 8,
            'headroom_height': 0,
            'ceiling_height': 8,
            'aisle_width': 0.5,
            'seat_distribution': [1],
            'cargo': True
        },
        {
            'seat_width': 1.5,
            'seat_depth': 2.6,
            'seat_height': 4,
            'headroom_height': 1,
            'ceiling_height': 7,
            'aisle_width': 2,
            'seat_distribution': [3, 6, 3]
        },
        {
            'seat_width': 1.5,
            'seat_depth': 2.6,
            'seat_height': 4,
            'headroom_height': 1,
            'ceiling_height': 7,
            'aisle_width': 2,
            'seat_distribution': [3, 5, 3]
        },
        {
            'seat_width': 2.5,
            'seat_depth': 2.6,
            'seat_height': 4,
            'headroom_height': 1,
            'ceiling_height': 7,
            'aisle_width': 2,
            'seat_distribution': [1, 2, 1]
        }
    ]

    # Create aircraft
    aircraft = Aircraft("large_passenger")

    # Add fuselage
    aircraft.add_fuselage(
        total_passengers=1250,
        seat_configs=seat_configs,
        galleys=[
            {'length': 20, 'number': 2, 'floor': 1},
        ],
        cockpit_length=20,
        cargo_per_passenger=10,
        nose_fineness=3.5,    # Longer, more pointed nose
        tail_length=30,       # 40 feet long tail section
        tail_grade=1.5        # Sharp initial taper that gradually levels off
    )
    fuselage_height = aircraft.fuselage.geometry.parameters['max_height']
    fuselage_width = aircraft.fuselage.geometry.parameters['max_width']
    fuselage_length = aircraft.fuselage.geometry.parameters['length']

    # Add main wing
    aircraft.add_wing(
        chord_points=[(150, 0), (105, .3), (80, .5), (30, .8), (10, 1)],  # (chord, span_location)
        half_span=275/2,
        thicknesses=[fuselage_height, fuselage_height*.8,fuselage_height*.8,fuselage_height*.5, 10*0.12],
        sweep=40,
        dihedral=0,
        position=Position(x=20, y=0, z=fuselage_height/2)
    )
    # Print the area of the wing
    wing_area = aircraft.children[-1].geometry.area
    print(f"Wing Area: {wing_area} square feet")


    # Add vertical tail
    aircraft.add_tail(
        half_span=20,  
        root_chord=35,  
        tip_chord=20,   
        sweep=35,
        tail_type='vertical',
        position=Position(x=fuselage_length*.8, y=10, z=fuselage_height/3),
        cant_angle=20  # 15 degrees outward cant
    )
    aircraft.add_tail(
        half_span=20,  
        root_chord=35,  
        tip_chord=20,   
        sweep=35,
        tail_type='vertical',
        position=Position(x=fuselage_length*.8, y=-10, z=fuselage_height/3),
        cant_angle=-20  # 15 degrees outward cant
    )

    # Add engines
    aircraft.add_engines([{
        'radius': 5,
        'length': 20,
        'positions': [
            (30, 50, -15),   # Adjusted X position to be relative to wing
            (70, 100, -15)   # Adjusted X position to be relative to wing
        ]
    }])

    return aircraft

if __name__ == "__main__":
    aircraft = main()
    print(f"Wing Area: {aircraft.wing.geometry.area} square feet")
    print(f"Aspect Ratio: {aircraft.wing.geometry.aspect_ratio}")
    print(f"Volume: {aircraft.wing.geometry.volume} cubic feet")
    """
    print("\nAircraft Components:")
    for child in aircraft.children:
        print(f"- {child.name}")
        if hasattr(child, 'geometry') and child.geometry:
            print("  Parameters:")
            for key, value in child.geometry.parameters.items():
                print(f"    {key}: {value}")

    print("\nAircraft Performance Analysis:")
    analysis = aircraft.analyze_performance()
    for category, results in analysis.items():
        print(f"\n{category.capitalize()} Analysis:")
        for result, value in results.items():
            print(f"- {result}: {value}") 
    """
    # Create a 3D view
    #fig, ax = aircraft.plot_3d()
    plt.show()

    # Create three-view drawing of the wing only
    fig, (ax_top, ax_side, ax_front) = aircraft.plot_views()
    #fig, ax_top = aircraft.plot_views()
    plt.show()
