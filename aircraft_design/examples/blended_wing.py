from typing import List, Tuple, Dict, Optional
import numpy as np
import matplotlib.pyplot as plt
import copy
from aircraft_design.components.interior.blended_wing_cabin import SingleFloorCabin, EconomyBlock, Galley, Bathroom
from aircraft_design.components.interior.base import InteriorComponent
from aircraft_design.components.fuselage.fuselage_geometry import FuselageGeometry, CrossSection, CrossSectionShape
from aircraft_design.components.aerodynamics.basic_aero import AerodynamicComponent
from aircraft_design.core.base import Component, Position
from aircraft_design.core.plotting import Object3D, plot_orthographic_views, plot_cross_section
from aircraft_design.components.aerodynamics.wing_geometry import TrailingEdgeWingGeometry, TailGeometry
from aircraft_design.components.aerodynamics.basic_aero import AverageThicknessAnalysis, ParasiticDragAnalysis, OswaldEfficiencyAnalysis
class CargoFloor(InteriorComponent):
    """Basic cargo floor component"""
    def __init__(self, name: str, length: float, width: float, height: float = 8.0):
        super().__init__(name)
        self.geometry.parameters.update({
            'length': length,
            'width': width,
            'height': height
        })

def create_passenger_floor(name: str, config: Dict[str, Dict[str, List[int]]] = None) -> SingleFloorCabin:
    """Create a single passenger floor with standard layout
    
    Args:
        name: Name of the floor
        config: Optional dictionary containing seating configuration for each section:
            {
                'front': {'seat_groups': [3,4,3], 'rows_per_section': [4,4]},
                'middle': {'seat_groups': [3,6,3], 'rows_per_section': [4,5,4]},
                'rear': {'seat_groups': [3,6,3], 'rows_per_section': [6,8]}
            }
            If not provided, uses default configuration.
    """
    cabin = SingleFloorCabin(name)
    
    # Default configurations
    default_config = {
        'front': {
            'seat_groups': [3, 4, 3],
            'rows_per_section': [4, 4]
        },
        'middle': {
            'seat_groups': [3, 6, 3],
            'rows_per_section': [4, 5, 4]
        },
        'rear': {
            'seat_groups': [3, 6, 3],
            'rows_per_section': [6, 8]
        }
    }
    
    # Merge provided config with defaults
    if config is None:
        config = default_config
    else:
        for section in ['front', 'middle', 'rear']:
            if section not in config:
                config[section] = default_config[section]
            else:
                # Ensure all required keys exist
                for key in ['seat_groups', 'rows_per_section']:
                    if key not in config[section]:
                        config[section][key] = default_config[section][key]
    
    # Create front economy block
    front_economy = EconomyBlock(
        name=f"{name}_front_economy",
        seat_groups=config['front']['seat_groups'],
        rows_per_section=config['front']['rows_per_section'],
        seats_per_exit=60
    )
    
    # Create middle economy block
    middle_economy = EconomyBlock(
        name=f"{name}_middle_economy",
        seat_groups=config['middle']['seat_groups'],
        rows_per_section=config['middle']['rows_per_section'],
        seats_per_exit=60
    )

    # Create rear economy block
    rear_economy = EconomyBlock(
        name=f"{name}_rear_economy",
        seat_groups=config['rear']['seat_groups'],
        rows_per_section=config['rear']['rows_per_section'],
        seats_per_exit=60
    )
    
    # Create galleys and bathrooms
    GALLEY_SPACING = 2.0
    mid_galley = Galley(f"{name}_mid_galley", width=10.0, depth=4.0, height=7.0)
    rear_galley = Galley(f"{name}_rear_galley", width=15.0, depth=4.0, height=7.0)
    
    bathroom_width = 3.0
    bathroom_depth = 3.0
    mid_bathroom = Bathroom(f"{name}_mid_bathroom", width=bathroom_width, depth=bathroom_depth)
    back_bathroom = Bathroom(f"{name}_back_bathroom", width=bathroom_width, depth=bathroom_depth)
    
    # Add components to cabin in sequence
    current_x = 0.0
    
    cabin.add_component(front_economy, current_x)
    current_x += front_economy.total_length + GALLEY_SPACING
    
    cabin.add_component(mid_galley, current_x)
    cabin.add_component(mid_bathroom, current_x)
    current_x += mid_galley.geometry.parameters['depth'] + GALLEY_SPACING
    
    cabin.add_component(middle_economy, current_x)
    current_x += middle_economy.total_length + GALLEY_SPACING
    
    cabin.add_component(rear_economy, current_x)
    current_x += rear_economy.total_length + GALLEY_SPACING
    
    cabin.add_component(rear_galley, current_x)
    cabin.add_component(back_bathroom, current_x)
    
    return cabin, [front_economy, middle_economy, rear_economy]

class BlendedWingFuselage(Component):
    """Top level aircraft class for blended wing design"""
    def __init__(self, name: str):
        super().__init__(name)
        self._components = {}

    def create_fuselage(self, first_floor: SingleFloorCabin, second_floor: SingleFloorCabin, cargo_floor: CargoFloor,
                       floor_blocks: List[List[EconomyBlock]], floor_height: float = 8.0,
                       tail_length: float = 10.0,
                       nose_length: float = 10.0) -> None:
        """Create and add fuselage component with floors"""
        # Create fuselage component
        self.fuselage = AerodynamicComponent("fuselage")
        
        # Create cross-sections for fuselage geometry
        cross_sections = []
        

        # Add points at each economy block transition
        bounding_boxes = []  # x, y0,z0, y1,z1



        # Add nose bounding box
        # Add nose section
        nose_height = 10.0
        nose_width = 10.0
        cross_sections.append((-nose_length, nose_width, nose_height, nose_height/2-floor_height))  # Narrow nose
        
        # Process each floor's blocks with their correct z-offset
        floor_configs = [
            (floor_blocks[0], 0.0),           # First floor at z=0
            (floor_blocks[1], floor_height),  # Second floor above
        ]
        
        for blocks, z_offset in floor_configs:
            for block in blocks:
                # Get block's position in global coordinates
                pos = block.get_global_position()
                width = block.total_width
                
                # Add bounding box at start of block
                x_start = pos[0]
                y0 = pos[1] - width/2
                y1 = pos[1] + width/2
                z0 = z_offset - floor_height/2
                z1 = z_offset + floor_height/2
                bounding_boxes.append([x_start, y0, z0, y1, z1])
                
                # Add bounding box at end of block
                x_end = x_start + block.total_length
                bounding_boxes.append([x_end, y0, z0, y1, z1])
        
        # Add cargo floor bounding box
        cargo_width = cargo_floor.geometry.parameters['width']
        cargo_pos = cargo_floor.get_global_position()
        bounding_boxes.append([
            cargo_pos[0],
            cargo_pos[1] - cargo_width/2,
            -floor_height - floor_height/2,
            cargo_pos[1] + cargo_width/2,
            -floor_height + floor_height/2
        ])
        bounding_boxes.append([
            cargo_pos[0] + cargo_floor.geometry.parameters['length'],
            cargo_pos[1] - cargo_width/2,
            -floor_height - floor_height/2,
            cargo_pos[1] + cargo_width/2,
            -floor_height + floor_height/2
        ])
        
        # Convert to numpy array for easier manipulation
        bounding_boxes = np.array(bounding_boxes)
        
        # Sort bounding boxes by x coordinate
        sort_idx = np.argsort(bounding_boxes[:, 0])
        bounding_boxes = bounding_boxes[sort_idx]
        
        # Remove duplicates at same x-coordinate, taking min/max y coordinates
        unique_x = []
        unique_boxes = []
        current_x = None
        current_box = None
        
        for box in bounding_boxes:
            x = box[0]
            if x != current_x:
                if current_box is not None:
                    unique_boxes.append(current_box)
                current_x = x
                current_box = box.copy()  # Make a copy to modify
            else:
                # Take minimum y0 (leftmost) and maximum y1 (rightmost)
                current_box[1] = min(current_box[1], box[1])  # min y0
                current_box[2] = min(current_box[2], box[2])  # min z0
                current_box[3] = max(current_box[3], box[3])  # max y1
                current_box[4] = max(current_box[4], box[4])  # max z1
        
        # Don't forget to append the last box
        if current_box is not None:
            unique_boxes.append(current_box)
        
        # Convert back to numpy array
        bounding_boxes = np.array(unique_boxes)
        
        # Find indices of extrema
        min_y0_idx = np.argmin(bounding_boxes[:, 1])  # Column 1 is y0
        min_z0_idx = np.argmin(bounding_boxes[:, 2])  # Column 2 is z0
        max_y1_idx = np.argmax(bounding_boxes[:, 3])  # Column 3 is y1
        max_z1_idx = np.argmax(bounding_boxes[:, 4])  # Column 4 is z1
        
        # Get the x coordinates where these extrema occur
        min_y0_x = bounding_boxes[min_y0_idx, 0]
        max_y1_x = bounding_boxes[max_y1_idx, 0]
        min_z0_x = bounding_boxes[min_z0_idx, 0]
        max_z1_x = bounding_boxes[max_z1_idx, 0]
        
        corrected_bounding_boxes = bounding_boxes.copy()
        
        # Forward pass: Ensure monotonic increase to maximum
        running_min_y0 = float('inf')
        running_min_z0 = float('inf')
        running_max_y1 = float('-inf')
        running_max_z1 = float('-inf')
        
        for i in range(len(bounding_boxes)):
            x = bounding_boxes[i, 0]
            
            # Update running extrema
            running_min_y0 = min(running_min_y0, bounding_boxes[i, 1])
            running_min_z0 = min(running_min_z0, bounding_boxes[i, 2])
            running_max_y1 = max(running_max_y1, bounding_boxes[i, 3])
            running_max_z1 = max(running_max_z1, bounding_boxes[i, 4])
            
            # Apply running values up to maximum points
            if x <= min_y0_x:
                corrected_bounding_boxes[i, 1] = running_min_y0
            if x <= min_z0_x:
                corrected_bounding_boxes[i, 2] = running_min_z0
            if x <= max_y1_x:
                corrected_bounding_boxes[i, 3] = running_max_y1
            if x <= max_z1_x:
                corrected_bounding_boxes[i, 4] = running_max_z1
        
        # Backward pass: Ensure monotonic decrease after maximum
        running_min_y0 = float('inf')
        running_min_z0 = float('inf')
        running_max_y1 = float('-inf')
        running_max_z1 = float('-inf')
        
        for i in range(len(bounding_boxes)-1, -1, -1):
            x = bounding_boxes[i, 0]
            
            # Update running extrema
            running_min_y0 = min(running_min_y0, bounding_boxes[i, 1])
            running_min_z0 = min(running_min_z0, bounding_boxes[i, 2])
            running_max_y1 = max(running_max_y1, bounding_boxes[i, 3])
            running_max_z1 = max(running_max_z1, bounding_boxes[i, 4])
            
            # Apply running values after maximum points
            if x > min_y0_x:
                corrected_bounding_boxes[i, 1] = running_min_y0
            if x > min_z0_x:
                corrected_bounding_boxes[i, 2] = running_min_z0
            if x > max_y1_x:
                corrected_bounding_boxes[i, 3] = running_max_y1
            if x > max_z1_x:
                corrected_bounding_boxes[i, 4] = running_max_z1
        
        # Create cross sections from corrected bounding boxes
        for bb in corrected_bounding_boxes:
            x = bb[0]
            width = bb[3] - bb[1]
            height = bb[4] - bb[2]
            z_offset = (bb[2] + bb[4]) / 2 + floor_height/2
            cross_sections.append((x, width, height, z_offset))
        
        # Add tail section
        tail_height = 2.0
        tail_width = 5.0
        cross_sections.append((x + tail_length, tail_width, tail_height, z_offset+height/2-tail_height/2))  # Narrow tail, slight upward sweep
        cross_sections = np.array(cross_sections)

        self.max_width = max(width for x, width, height, z_offset in cross_sections)
        self.max_height = max(height for x, width, height, z_offset in cross_sections)
        self.max_length = max(x for x, width, height, z_offset in cross_sections)
        # Create fuselage geometry
        fuselage_geom = FuselageGeometry()

        # add the nose
        fuselage_geom.add_section(CrossSection(
            station=-1.2*nose_length,
            width=0.2,
            height=0.2,
        ))
        # Add cross-sections with super-ellipse shape
        for x, width, height, z_offset in cross_sections:
            section = CrossSection(
                station=x,
                width=width,
                height=height,
                shape=CrossSectionShape.SUPER_ELLIPSE,
                z_offset=z_offset,
                parameters={'n': 2.5}  # Super-ellipse parameter for smooth blending
            )
            fuselage_geom.add_section(section)
        
        # Set the geometry
        self.fuselage.geometry = fuselage_geom
        
        # Add floors as children of fuselage
        self.fuselage.add_child(first_floor)
        self.fuselage.add_child(second_floor)
        self.fuselage.add_child(cargo_floor)
        
        # Add fuselage as child of aircraft
        self.add_child(self.fuselage)
        self._components['fuselage'] = self.fuselage

    def plot(self) -> Object3D:
        """Create a 3D visualization of the entire aircraft"""
        obj = Object3D()
        
        # Plot fuselage in black
        if 'fuselage' in self._components:
            fuselage_obj = self._components['fuselage'].plot(plot_children=True)
            for shape in fuselage_obj.shapes:
                shape.metadata['color'] = 'black'
            obj.shapes.extend(fuselage_obj.shapes)
        
        return obj

class BlendedWingPlane(Component):
    """Top level aircraft class that manages all components"""
    def __init__(self, name: str):
        super().__init__(name)
        self._components = {}

    def add_wing(self,
                 chord_points: List[Tuple[float, float]],
                 half_span: float,
                 thicknesses: List[float],
                 te_sweep: float = 0.0,
                 dihedral: float = 0.0,
                 position: Optional[Position] = None) -> None:
        """Add main wing with given parameters"""
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
        
        self.wing = TrailingEdgeWingGeometry(waypoints)
        wing_component = AerodynamicComponent("main_wing")
        wing_component.geometry = self.wing
        wing_component.geometry.position = position if position else Position(0, 0, 0)
        
        # Update parameters which will automatically create waypoints
        self.wing.update_parameters({
            'span': total_span,
            'te_sweep': te_sweep,
            'dihedral': dihedral,
            'root_chord': chord_points[0][0],
            'tip_chord': chord_points[-1][0]
        })
        
        # Add wing as child of aircraft
        self.add_child(wing_component)
        self._components['wing'] = wing_component

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
            
        # Add the tail component directly to the aircraft
        self.add_child(tail_component)
        self._components[name] = tail_component

    def add_engines(self, engine_configs: List[Dict]) -> None:
        """Add engines with given configurations
        
        Args:
            engine_configs: List of dictionaries containing engine parameters:
                {
                    'radius': float,  # Engine radius in feet
                    'length': float,  # Engine length in feet
                    'positions': List[Tuple[float, float, float]]  # List of (x, y, z) positions
                }
        """
        from aircraft_design.components.propulsion.engine import Engine, EngineGeometry
        
        # Get the wing component for reference
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
                self._components[f'engine_{idx}_pos_{pos_idx}'] = engine_instance
                self.add_child(engine_instance)
                
                # Create secondary side engine (mirrored)
                if y != 0:  # Only create mirrored engine if not on centerline
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
                    self._components[f'engine_{idx}_pos_{pos_idx + 1}'] = engine_instance
                    self.add_child(engine_instance)

    def plot(self) -> Object3D:
        """Create a 3D visualization of the entire aircraft"""
        obj = Object3D()
        
        # Plot all components with their colors
        colors = {
            'fuselage': 'black',
            'main_wing': 'blue'
        }
        
        for child in self.children:
            child_obj = child.plot()
            for shape in child_obj.shapes:
                shape.metadata['color'] = colors.get(child.name, 'gray')
            obj.shapes.extend(child_obj.shapes)
        
        return obj

def main(plot: bool = True):
    # Create top-level plane component
    plane = BlendedWingPlane("blended_wing_plane")
    
    # Create aircraft fuselage component
    fuselage = BlendedWingFuselage("blended_wing")
    
    # Example of custom configuration for first floor
    first_floor_config = {
        'front': {
            'seat_groups': [3, 4, 3],
            'rows_per_section': [4, 4]  # More rows in front sections
        },
        'middle': {
            'seat_groups': [3, 6, 3],
            'rows_per_section': [4, 6, 6]  # Adjusted middle section
        },
        'rear': {
            'seat_groups': [3, 6, 3],
            'rows_per_section': [8, 8]
        }
    }
    
    # Example of custom configuration for second floor
    second_floor_config = {
        'front': {
            'seat_groups': [3, 6, 3],  # Different seat group configuration
            'rows_per_section': [8, 8]
        },
        'middle': {
            'seat_groups': [3, 6,6, 3],
            'rows_per_section': [6, 6, 6]
        },
        'rear': {
            'seat_groups': [3,6,6,6,3],
            'rows_per_section': [4, 4, 4,]
        }
    }
    
    # Create first passenger floor with custom config
    first_floor, first_floor_blocks = create_passenger_floor("first_floor", first_floor_config)
    
    # Create second passenger floor with different custom config
    second_floor, second_floor_blocks = create_passenger_floor("second_floor", second_floor_config)
    
    # Position second floor above first floor
    FLOOR_HEIGHT = 8.0  # 8 feet between floors
    second_floor.geometry.position.z = FLOOR_HEIGHT
    
    # Create cargo floor
    cargo_height = 9.0
    cargo_width = 30.0
    cargo_length = (1255*10 + 1.8e4)/cargo_width/cargo_height*1.2
    cargo_floor = CargoFloor(
        "cargo_floor",
        width=cargo_width,
        height=cargo_height,
        length=cargo_length,
    )
    print(f"Cargo floor length: {cargo_floor.geometry.parameters['length']}")
    cargo_floor.geometry.position.z = -FLOOR_HEIGHT  # Position below first floor
    floor_blocks = [first_floor_blocks, second_floor_blocks]
    
    # Create fuselage with floors
    fuselage.create_fuselage(
        first_floor=first_floor,
        second_floor=second_floor,
        cargo_floor=cargo_floor,
        floor_blocks=floor_blocks,
        floor_height=FLOOR_HEIGHT,
        tail_length=20.0,
        nose_length=30.0
    )
    
    # Add the fuselage component to the plane
    plane.add_child(fuselage)
    
    # Add wing to the plane
    fuselage_height = FLOOR_HEIGHT * 3  # Total height of all floors
    plane.add_wing(
        chord_points=[(150, 0), (90, 0.25), (40, 0.5), (10, 1.0)],
        half_span=300/2,
        thicknesses=[fuselage_height*0.8, fuselage_height*.6, 60*.2, 30*.1],
        te_sweep=5,
        dihedral=3,
        position=Position(x=0, y=0, z=fuselage_height/2)
    )

    # Add vertical tails
    # Left vertical tail
    plane.add_tail(
        half_span=40,  
        root_chord=35,  
        tip_chord=20,   
        sweep=35,
        tail_type='vertical',
        position=Position(x=fuselage.max_length*.8, y=10, z=fuselage_height/3),
        cant_angle=30  # 30 degrees outward cant
    )
    # Right vertical tail
    plane.add_tail(
        half_span=40,  
        root_chord=35,  
        tip_chord=20,   
        sweep=35,
        tail_type='vertical',
        position=Position(x=fuselage.max_length*.8, y=-10, z=fuselage_height/3),
        cant_angle=-30  # -30 degrees outward cant
    )

    # Add engines
    plane.add_engines([{
        'radius': 5.,
        'length': 20,
        'positions': [
            (150, 0, 5),   # Center engine
            (150, 25, 7),  # Outboard engine
        ]
    }])
    
    # Print statistics
    print("\nBlended Wing Aircraft Statistics:")
    print(f"Total Length: {fuselage.max_length:.1f} ft")
    print(f"Maximum Width: {fuselage.max_width:.1f} ft")
    print(f"Total Height: {FLOOR_HEIGHT * 3:.1f} ft")
    print(f"Fuselage Volume: {fuselage.fuselage.geometry.volume:.1f} ft^3")
    print(f"Useable Fuel Volume Estimate: {plane.wing.volume - fuselage.fuselage.geometry.volume:.1f} ft^3")
    print(f"First Floor Seats: {sum(block.num_seats for block in first_floor_blocks)}")
    print(f"Second Floor Seats: {sum(block.num_seats for block in second_floor_blocks)}")
    print(f"Total Passenger Seats: {sum(block.num_seats for block in first_floor_blocks) + sum(block.num_seats for block in second_floor_blocks)}")
    print(f"Wing Area: {plane.wing.area:.1f} ft^2")
    print(f"Wing Volume: {plane.wing.volume:.1f} ft^3")
    print(f"Wing AR: {plane.wing.aspect_ratio:.1f}")

    # run thickness analysis for wing
    average_thickness_analysis = AverageThicknessAnalysis()
    average_thickness = average_thickness_analysis.run(plane.wing)
    print(f"Average Thickness: {average_thickness['average_thickness']:.1f} ft")
    print(f"Average TC Ratio: {average_thickness['average_tc_ratio']:.1f}")

    # Analyze aircraft performance
    results = analyze_performance(plane)
    
    # Print performance results
    print("\nAircraft Performance Analysis:")
    for category, category_results in results.items():
        print(f"\n{category.capitalize()} Analysis:")
        for key, value in category_results.items():
            try:
                print(f"- {key}: {value:.4f}")
            except:
                print(f"- {key}: {value}")

    if plot:
        # Create three-view drawing using the plane component
        plane_obj = plane.plot()
        fig = plt.figure(figsize=(15, 10))
        fig, (ax_top, ax_side, ax_front) = plot_orthographic_views(plane_obj, fig=fig)
        plt.suptitle('Blended Wing Aircraft', fontsize=16)
        plt.tight_layout()
        plt.savefig('assets/blended_wing_aircraft.png', dpi=300, bbox_inches='tight')
        plt.close('all')

        # Create x-axis cross sections at key stations
        fuselage_obj = fuselage.plot()
        stations = [25, 75, 125]  # Key x-coordinates for cross sections: nose, front cabin, mid cabin, rear
        fig = plt.figure(figsize=(15, 10))
        for i, station in enumerate(stations, 1):
            ax = fig.add_subplot(2, 2, i)
            plot_cross_section(fuselage_obj, plane='x', value=station, height=5.0, ax=ax,use_opacity=False)
            ax.set_title(f'Station x={station} ft')
            ax.set_xlim(-30, 30)  # Set x limits
            ax.set_ylim(-10, 20)  # Set y limits
        plt.suptitle('Fuselage Cross Sections (X-axis)', fontsize=16)
        plt.tight_layout()
        plt.savefig('assets/x_cross_sections.png', dpi=300, bbox_inches='tight')
        plt.close('all')

        # Create z-axis cross sections at passenger floors
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(15, 8))
        
        # First floor cross section
        plot_cross_section(fuselage_obj, plane='z', value=FLOOR_HEIGHT/4, height=FLOOR_HEIGHT/4, ax=ax1,use_opacity=False)
        ax1.set_title('First Floor (z=0 ft)')
        
        # Second floor cross section
        plot_cross_section(fuselage_obj, plane='z', value=FLOOR_HEIGHT*5/4, height=FLOOR_HEIGHT/4, ax=ax2,use_opacity=False)
        ax2.set_title('Second Floor (z=8 ft)')
        
        plt.suptitle('Passenger Floor Cross Sections (Z-axis)', fontsize=16)
        plt.tight_layout()
        plt.savefig('assets/z_cross_sections.png', dpi=300, bbox_inches='tight')
        plt.close('all')

        # create a y cross section view of the fuselage
        fig, ax = plt.subplots(figsize=(15, 10))
        plot_cross_section(fuselage_obj, plane='y', value=-10, height=20, ax=ax,use_opacity=True)
        ax.set_title('Fuselage Cross Section (Y-axis)')
        plt.savefig('assets/y_cross_section.png', dpi=300, bbox_inches='tight')
        plt.close('all')

def analyze_performance(aircraft: BlendedWingPlane) -> Dict[str, Dict[str, float]]:
    """Analyze the performance of the blended wing aircraft
    
    Args:
        aircraft: The blended wing aircraft to analyze
        
    Returns:
        Dictionary containing analysis results for each component
    """
    results = {
        'aircraft': {},
        'wing': {},
        'fuselage': {},
        'tail': {},
        'engines': {}
    }
    
    # Get main components
    wing = next((c for c in aircraft.children if isinstance(c, AerodynamicComponent) and c.name == "main_wing"), None)
    fuselage = next((c for c in aircraft.children if isinstance(c, AerodynamicComponent) and c.name == "fuselage"), None)
    vtails = [c for c in aircraft.children if isinstance(c, AerodynamicComponent) and c.name == "vertical_tail"]
    engines = [c for c in aircraft.children if c.name.startswith("engine_")]
    
    # Get wing reference area for drag coefficients
    wing_area = wing.geometry.area if wing else 1.0
    
    # Wing analysis
    if wing:
        # Parasitic drag analysis
        drag_analysis = ParasiticDragAnalysis()
        drag_analysis.parameters.update({
            'mach': 0.85,  # cruise Mach number
            'reynolds': 1e7,  # approximate Reynolds number
            'characteristic_length': wing.geometry.mean_chord,
            'wetted_area': wing.geometry.wetted_area,
            'form_factor': 1.2  # Typical wing form factor
        })
        try:
            wing_drag = drag_analysis.run(wing)
            results['wing'].update({
                'CD0': wing_drag['CD0_component'] / wing_area,
                'Cf': wing_drag['Cf'],
                'wetted_area': wing_drag['wetted_area']
            })
        except Exception as e:
            print(f"Warning: Wing drag analysis failed: {e}")
            results['wing']['CD0'] = 0.0
        
        # Oswald efficiency analysis
        oswald_analysis = OswaldEfficiencyAnalysis()
        oswald_analysis.parameters.update({
            'mach': 0.9
        })
        try:
            oswald_results = oswald_analysis.run(wing)
            results['wing']['e'] = oswald_results['oswald_efficiency']
            results['wing']['AR'] = oswald_results['aspect_ratio']
        except Exception as e:
            print(f"Warning: Oswald efficiency analysis failed: {e}")
            results['wing']['e'] = 0.85  # Default value
    
    # Fuselage analysis
    if fuselage:
        # Parasitic drag analysis
        drag_analysis = ParasiticDragAnalysis()
        drag_analysis.parameters.update({
            'mach': 0.9,
            'reynolds': 1e7,
            'characteristic_length': fuselage.geometry.parameters['length'],
            'wetted_area': fuselage.geometry.wetted_area,
            'form_factor': 1.15  # Typical fuselage form factor
        })
        try:
            fuselage_drag = drag_analysis.run(fuselage)
            results['fuselage'].update({
                'CD0': fuselage_drag['CD0_component'] / wing_area,
                'Cf': fuselage_drag['Cf'],
                'wetted_area': fuselage_drag['wetted_area']
            })
        except Exception as e:
            print(f"Warning: Fuselage drag analysis failed: {e}")
            results['fuselage']['CD0'] = 0.0
        
        # Add geometric properties
        results['fuselage'].update({
            'wetted_area': fuselage.geometry.wetted_area,
            'volume': fuselage.geometry.volume
        })

    # Vertical tail analysis
    for i, vtail in enumerate(vtails):
        # Parasitic drag analysis
        drag_analysis = ParasiticDragAnalysis()
        drag_analysis.parameters.update({
            'mach': 0.85,
            'reynolds': 1e7,
            'characteristic_length': vtail.geometry.mean_chord,
            'wetted_area': vtail.geometry.wetted_area,
            'form_factor': 1.1  # Typical tail form factor
        })
        try:
            vtail_drag = drag_analysis.run(vtail)
            if f'vertical_{i+1}' not in results['tail']:
                results['tail'][f'vertical_{i+1}'] = {}
            results['tail'][f'vertical_{i+1}'].update({
                'CD0': vtail_drag['CD0_component'] / wing_area,
                'Cf': vtail_drag['Cf'],
                'wetted_area': vtail_drag['wetted_area']
            })
        except Exception as e:
            print(f"Warning: Vertical tail {i+1} drag analysis failed: {e}")
            if f'vertical_{i+1}' not in results['tail']:
                results['tail'][f'vertical_{i+1}'] = {}
            results['tail'][f'vertical_{i+1}']['CD0'] = 0.0

    # Engine analysis
    for i, engine in enumerate(engines):
        # Engine drag
        drag_analysis = ParasiticDragAnalysis()
        drag_analysis.parameters.update({
            'mach': 0.85,
            'reynolds': 1e7,
            'characteristic_length': engine.geometry.parameters['length'],
            'wetted_area': engine.geometry.wetted_area,
            'form_factor': 1.3  # Typical nacelle form factor
        })
        try:
            engine_drag = drag_analysis.run(engine)
            results['engines'][f'engine_{i+1}'] = {
                'CD0': engine_drag['CD0_component'] / wing_area,
                'Cf': engine_drag['Cf'],
                'wetted_area': engine_drag['wetted_area']
            }
        except Exception as e:
            print(f"Warning: Engine {i+1} drag analysis failed: {e}")
            results['engines'][f'engine_{i+1}'] = {'CD0': 0.0}
    
    # Calculate total aircraft CD0
    total_cd0 = (
        results['wing'].get('CD0', 0.0) +
        results['fuselage'].get('CD0', 0.0) +
        sum(tail.get('CD0', 0.0) for tail in results['tail'].values()) +
        sum(engine.get('CD0', 0.0) for engine in results['engines'].values())
    )
    results['aircraft']['CD0'] = total_cd0
    
    # Calculate induced drag factor
    if 'e' in results['wing'] and 'AR' in results['wing']:
        k = 1 / (np.pi * results['wing']['e'] * results['wing']['AR'])
        results['aircraft']['K'] = k
    
    return results

if __name__ == "__main__":
    main(plot=True)
