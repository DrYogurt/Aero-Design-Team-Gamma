from typing import List, Dict, Any, Optional, Tuple, Union
from aircraft_design.core.base import Component, Position
from aircraft_design.components.aerodynamics.basic_aero import AerodynamicComponent
from aircraft_design.components.fuselage.fuselage_geometry import FuselageGeometry, CrossSection
from aircraft_design.components.aerodynamics.wing_geometry import WaypointWingGeometry, TailGeometry, SimpleSweptWing, TrailingEdgeWingGeometry
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
        print(f"available_cargo_volume: {available_cargo_volume}")
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
                 te_sweep: float = 0.0,
                 dihedral: float = 0.0,
                 position: Optional[Position] = None) -> None:
        """Add main wing with given parameters, using chord points and thicknesses to generate waypoints
        
        Args:
            chord_points: List of (chord_length, span_fraction) tuples
            thicknesses: List of thickness values for each chord point
            half_span: Half-span length in feet
            te_sweep: Trailing edge sweep angle in degrees
            dihedral: Dihedral angle in degrees
            position: Position of the wing root
        """
        if len(chord_points) != len(thicknesses):
            raise ValueError("chord_points and thicknesses must have the same length")
        
        # Calculate the total span
        total_span = half_span * 2
        
        # Generate waypoints from chord points and thicknesses
        waypoints = []
        for (chord, span_fraction), thickness in zip(chord_points, thicknesses):
            waypoints.append({
                'span_fraction': span_fraction,
                'chord': chord,
                'thickness': thickness
            })
        
        wing = TrailingEdgeWingGeometry(waypoints)
        wing_component = AerodynamicComponent("main_wing")
        wing_component.geometry = wing
        wing_component.geometry.position = position if position else Position(0, 0, 0)
        
        # Update parameters which will automatically create waypoints
        wing.update_parameters({
            'span': total_span,
            'te_sweep': te_sweep,
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
                self.add_child(engine_instance)
                
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
                self.add_child(engine_instance)

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
        
        # Get wing reference area for drag coefficients
        wing_area = wing.geometry.area if wing else 1.0
        
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
                    'reynolds': 1e7,
                    'characteristic_length': wing.geometry.mean_chord,
                    'wetted_area': wing.geometry.wetted_area
                })
                try:
                    wing_drag = wing.run_analysis('parasitic_drag')
                    results['wing'].update({
                        'CD0': wing_drag['CD0_component'] / wing_area,
                        'Cf': wing_drag['Cf'],
                        'wetted_area': wing_drag['wetted_area']
                    })
                except Exception as e:
                    print(f"Warning: Wing drag analysis failed: {e}")
                    results['wing']['CD0'] = 0.0
                
            # Oswald efficiency analysis
            oswald_analysis = wing.analyses.get('oswald_efficiency')
            if oswald_analysis:
                try:
                    oswald_results = wing.run_analysis('oswald_efficiency')
                    results['wing']['e'] = oswald_results['oswald_efficiency']
                    results['wing']['AR'] = oswald_results['aspect_ratio']
                except Exception as e:
                    print(f"Warning: Oswald efficiency analysis failed: {e}")
                    results['wing']['e'] = 0.85  # Default value
            else:
                print("Warning: Oswald efficiency analysis not available")
                results['wing']['e'] = 0.85  # Default value

        # Fuselage analysis
        if fuselage:
            # Get fuselage drag
            drag_analysis = fuselage.analyses.get('parasitic_drag')
            if drag_analysis:
                drag_analysis.parameters.update({
                    'mach': 0.85,
                    'reynolds': 1e7,
                    'characteristic_length': fuselage.geometry.parameters['length'],
                    'wetted_area': fuselage.geometry.wetted_area,
                    'form_factor': 1.15  # Typical fuselage form factor
                })
                try:
                    fuselage_drag = fuselage.run_analysis('parasitic_drag')
                    results['fuselage'].update({
                        'CD0': fuselage_drag['CD0_component'] / wing_area,
                        'Cf': fuselage_drag['Cf'],
                        'wetted_area': fuselage_drag['wetted_area']
                    })
                except Exception as e:
                    print(f"Warning: Fuselage drag analysis failed: {e}")
                    results['fuselage']['CD0'] = 0.0

            results['fuselage'].update({
                'wetted_area': fuselage.geometry.wetted_area,
                'volume': fuselage.geometry.volume
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
            
            # Vertical tail drag
            drag_analysis = vtail.analyses.get('parasitic_drag')
            if drag_analysis:
                drag_analysis.parameters.update({
                    'mach': 0.85,
                    'reynolds': 1e7,
                    'characteristic_length': vtail.geometry.mean_chord,
                    'wetted_area': vtail.geometry.wetted_area,
                    'form_factor': 1.1  # Typical tail form factor
                })
                try:
                    vtail_drag = vtail.run_analysis('parasitic_drag')
                    if 'vertical' not in results['tail']:
                        results['tail']['vertical'] = {}
                    results['tail']['vertical'].update({
                        'CD0': vtail_drag['CD0_component'] / wing_area,
                        'Cf': vtail_drag['Cf'],
                        'wetted_area': vtail_drag['wetted_area']
                    })
                except Exception as e:
                    print(f"Warning: Vertical tail drag analysis failed: {e}")
                    if 'vertical' not in results['tail']:
                        results['tail']['vertical'] = {}
                    results['tail']['vertical']['CD0'] = 0.0
        
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
            
            # Horizontal tail drag
            drag_analysis = htail.analyses.get('parasitic_drag')
            if drag_analysis:
                drag_analysis.parameters.update({
                    'mach': 0.85,
                    'reynolds': 1e7,
                    'characteristic_length': htail.geometry.mean_chord,
                    'wetted_area': htail.geometry.wetted_area,
                    'form_factor': 1.1  # Typical tail form factor
                })
                try:
                    htail_drag = htail.run_analysis('parasitic_drag')
                    results['tail']['horizontal'].update({
                        'CD0': htail_drag['CD0_component'] / wing_area,
                        'Cf': htail_drag['Cf'],
                        'wetted_area': htail_drag['wetted_area']
                    })
                except Exception as e:
                    print(f"Warning: Horizontal tail drag analysis failed: {e}")
                    results['tail']['horizontal']['CD0'] = 0.0
        
        # Engine analysis
        for idx, engine in enumerate(engines):
            engine_analysis = engine.analyses.get('thrust')
            if engine_analysis:
                try:
                    results['engines'][f'engine_{idx}'] = engine.run_analysis('thrust')
                except Exception as e:
                    print(f"Warning: Engine {idx} analysis failed: {e}")
                    results['engines'][f'engine_{idx}'] = {'thrust': 0.0}
            
            # Engine drag
            drag_analysis = engine.analyses.get('parasitic_drag')
            if drag_analysis:
                drag_analysis.parameters.update({
                    'mach': 0.85,
                    'reynolds': 1e7,
                    'characteristic_length': engine.geometry.parameters['length'],
                    'wetted_area': engine.geometry.wetted_area,
                    'form_factor': 1.3  # Typical nacelle form factor
                })
                try:
                    engine_drag = engine.run_analysis('parasitic_drag')
                    results['engines'][f'engine_{idx}'].update({
                        'CD0': engine_drag['CD0_component'] / wing_area,
                        'Cf': engine_drag['Cf'],
                        'wetted_area': engine_drag['wetted_area']
                    })
                except Exception as e:
                    print(f"Warning: Engine {idx} drag analysis failed: {e}")
                    results['engines'][f'engine_{idx}']['CD0'] = 0.0

        # Calculate total aircraft CD0
        total_cd0 = (
            results['wing'].get('CD0', 0.0) +
            results['fuselage'].get('CD0', 0.0) +
            results['tail'].get('vertical', {}).get('CD0', 0.0) +
            results['tail'].get('horizontal', {}).get('CD0', 0.0) +
            sum(engine.get('CD0', 0.0) for engine in results['engines'].values())
        )
        results['aircraft']['CD0'] = total_cd0

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
        total_passengers=1255,
        seat_configs=seat_configs,
        galleys=[
            {'length': 20, 'number': 2, 'floor': 1},
            {'length': 30, 'number': 1, 'floor': 1},
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
    print(f"Fuselage Height {fuselage_height}")
    print(f"Fuselage Width {fuselage_width}")
    print(f"Fuselage Length {fuselage_length}")
    # Add main wing
    aircraft.add_wing(
        chord_points=[(150, 0), (90, 0.25), (40, 0.5), (10, 1.0)],  # (chord, span_location)
        half_span=300/2,
        thicknesses=[fuselage_height, fuselage_height*.4, 40*.2, 30*.1],
        te_sweep=5,  # Trailing edge sweep angle
        dihedral=3,   # Slight dihedral
        position=Position(x=50, y=0, z=fuselage_height/2)  # Position relative to fuselage length
    )
    # Print the area of the wing
    wing_area = aircraft.children[-1].geometry.area
    print(f"Wing Area: {wing_area} square feet")


    # Add vertical tail
    aircraft.add_tail(
        half_span=40,  
        root_chord=35,  
        tip_chord=20,   
        sweep=35,
        tail_type='vertical',
        position=Position(x=fuselage_length*.8, y=10, z=fuselage_height/3),
        cant_angle=30  # 15 degrees outward cant
    )
    aircraft.add_tail(
        half_span=40,  
        root_chord=35,  
        tip_chord=20,   
        sweep=35,
        tail_type='vertical',
        position=Position(x=fuselage_length*.8, y=-10, z=fuselage_height/3),
        cant_angle=-30  # 15 degrees outward cant
    )

    # Add engines
    aircraft.add_engines([{
        'radius': 5.,
        'length': 20,
        'positions': [
            (150, 0, 5),   # Adjusted X position to be relative to wing
            (150, 25, 7)   # Adjusted X position to be relative to wing
        ]
    }])

    return aircraft

if __name__ == "__main__":
    aircraft = main()
    
    # Print geometric properties for main wing
    print("\nMain Wing Properties:")
    print(f"Area: {aircraft.wing.geometry.area:.2f} square feet")
    print(f"Aspect Ratio: {aircraft.wing.geometry.aspect_ratio:.2f}")
    print(f"Volume: {aircraft.wing.geometry.volume:.2f} cubic feet")
    print(f"Mean Chord: {aircraft.wing.geometry.mean_chord:.2f} feet")
    
    # Print geometric properties for vertical tails
    if hasattr(aircraft, 'vertical_tail'):
        if isinstance(aircraft.vertical_tail, list):
            for i, tail in enumerate(aircraft.vertical_tail):
                print(f"\nVertical Tail {i+1} Properties:")
                print(f"Area: {tail.geometry.area:.2f} square feet")
                print(f"Aspect Ratio: {tail.geometry.aspect_ratio:.2f}")
                print(f"Mean Chord: {tail.geometry.mean_chord:.2f} feet")
        else:
            print("\nVertical Tail Properties:")
            print(f"Area: {aircraft.vertical_tail.geometry.area:.2f} square feet")
            print(f"Aspect Ratio: {aircraft.vertical_tail.geometry.aspect_ratio:.2f}")
            print(f"Mean Chord: {aircraft.vertical_tail.geometry.mean_chord:.2f} feet")
    

    print("\nAircraft Performance Analysis:")
    analysis = aircraft.analyze_performance()
    for category, results in analysis.items():
        print(f"\n{category.capitalize()} Analysis:")
        for result, value in results.items():
            print(f"- {result}: {value}") 

    # Create three-view drawing
    fig, (ax_top, ax_side, ax_front) = aircraft.plot_views()
    plt.savefig('assets/large_passenger_aircraft.png')
    #plt.show()
    plt.close()