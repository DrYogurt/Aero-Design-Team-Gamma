from typing import List, Dict, Union, Optional
import numpy as np
from aircraft_design.components.interior.base import InteriorComponent
from aircraft_design.components.interior.seating import SeatingSection, SeatGeometry, Seat
from aircraft_design.components.interior.aisle import Aisle
from aircraft_design.components.interior.service import Galley, Bathroom
from aircraft_design.core.plotting import Object3D, Shape3D, create_box, plot_orthographic_views, plot_3d_object
from aircraft_design.analysis.mass_analysis import MassFeature, MassAnalysis

class EconomySection(SeatingSection):
    """Standard economy class section configuration"""
    
    # Default economy class measurements (in ft)
    DEFAULT_SPACING = {
        'seat_width': 18/12,  # 18 inches
        'seat_height': 39/12,  # 39 inches
        'seat_spacing': 31/12,  # 31 inches (center-to-center distance)
        'aisle_width': 20/12,  # 20 inches
        'seat_depth': 18/12   # 18 inches
    }

    def __init__(self, name: str, seat_groups: List[int], num_rows: int = 1, spacing: Dict[str, float] = None):
        """
        Initialize economy class section
        
        Args:
            name: Section name
            seat_groups: List of integers representing number of seats in each group
                        (e.g. [3,4,3] for a 3-4-3 configuration)
            num_rows: Number of rows in the section
            spacing: Dictionary of spacing measurements, defaults to DEFAULT_SPACING if None
        """
        super().__init__(name, section_type='economy')
        self.num_rows = num_rows
        self.seat_groups_config = seat_groups
        self.spacing = self.DEFAULT_SPACING.copy()
        if spacing:
            self.spacing.update(spacing)
        self._configure_rows()
    
    @property
    def num_seats(self) -> int:
        """Calculate total number of seats in the section"""
        return sum(self.seat_groups_config) * self.num_rows

    @property   
    def total_length(self) -> float:
        """Calculate total length of the section"""
        return self.num_rows * self.spacing['seat_spacing']

    @property
    def total_width(self) -> float:
        """Calculate total width of the section including aisles"""
        total_width = 0
        for i, num_seats in enumerate(self.seat_groups_config):
            total_width += num_seats * self.spacing['seat_width']
            if i < len(self.seat_groups_config) - 1:
                total_width += self.spacing['aisle_width']
        return total_width

    def _configure_rows(self):
        """Configure all seat groups and aisles based on the seat_groups configuration"""
        current_y = 0
        
        for group_idx, num_seats in enumerate(self.seat_groups_config):
            seats_in_group = []
            
            # Create seats for each row in this group
            for row in range(self.num_rows):
                for seat_idx in range(num_seats):
                    seat = Seat(f"{self.name}_seat_{row}_{group_idx}_{seat_idx}", seat_class='economy')
                    
                    # Configure seat geometry
                    width = self.spacing['seat_width']
                    depth = self.spacing['seat_depth']
                    height = self.spacing['seat_height']
                    
                    seat.geometry.parameters.update({
                        'width': width,
                        'depth': depth,
                        'height': height,
                        'seat_spacing': self.spacing['seat_spacing']
                    })
                    
                    # Set seat position
                    seat.geometry.position.x = row * self.spacing['seat_spacing']
                    seat.geometry.position.y = current_y + seat_idx * self.spacing['seat_width']
                    seat.geometry.position.z = 0
                    
                    # Set seat metadata
                    seat.row = row
                    seat.group = group_idx
                    
                    # Add mass properties - typical economy seat ~35 lbs and 205 lbs for passenger
                    mass = 35 + 205  # lbs
                    
                    # Calculate moments of inertia for a rectangular prism
                    # I = (1/12) * mass * (dimension^2)
                    ixx = (1/12) * mass * (height**2 + depth**2)
                    iyy = (1/12) * mass * (width**2 + height**2)
                    izz = (1/12) * mass * (width**2 + depth**2)
                    
                    # Create and add mass feature
                    mass_feature = MassFeature(
                        mass=mass,
                        center_of_gravity=[
                            seat.geometry.position.x + depth/2,
                            seat.geometry.position.y + width/2,
                            seat.geometry.position.z + height/2
                        ],
                        ixx=ixx, iyy=iyy, izz=izz,
                        ixy=0, ixz=0, iyz=0  # Assuming symmetrical mass distribution
                    )
                    seat.add_feature(mass_feature)
                    seat.add_analysis(MassAnalysis())
                    
                    seats_in_group.append(seat)
                    self.add_child(seat)
            
            self.seat_groups.append(seats_in_group)
            
            # Add aisle after each group except the last one
            if group_idx < len(self.seat_groups_config) - 1:
                current_y += num_seats * self.spacing['seat_width'] + self.spacing['aisle_width']
                self.add_aisle(self.spacing['aisle_width'], group_idx)
            else:
                current_y += num_seats * self.spacing['seat_width']

    def plot(self, color: str = 'lightblue', colors_dict: Dict[str, str] = None) -> Object3D:
        """Create a 3D visualization of the economy section"""
        obj = Object3D()
        
        # Plot all seats
        seat_color = colors_dict.get('seat', color) if colors_dict else color
        for group in self.seat_groups:
            for seat in group:
                seat_obj = seat.plot(color=seat_color)
                obj.shapes.extend(seat_obj.shapes)
        
        return obj

class ServiceComponent(InteriorComponent):
    """A service component that can contain galleys, bathrooms, and stairs"""
    
    def __init__(self, name: str, width: float, depth: float, height: float = 7.0):
        """
        Initialize a service component
        
        Args:
            name: Component name
            width: Total width of the service area
            depth: Total depth (length) of the service area
            height: Height of the service area
        """
        super().__init__(name)
        self.geometry.parameters.update({
            'width': width,
            'depth': depth,
            'height': height
        })
        self.sub_components = []
        
        # Add mass analysis
        self.add_analysis(MassAnalysis())
        
        # Add structural mass (~200 lbs for service area structure)
        mass = 200  # lbs
        # Moments of inertia for rectangular prism
        ixx = (1/12) * mass * (height**2 + depth**2)
        iyy = (1/12) * mass * (width**2 + height**2)
        izz = (1/12) * mass * (width**2 + depth**2)
        
        mass_feature = MassFeature(
            mass=mass,
            center_of_gravity=[depth/2, width/2, height/2],
            ixx=ixx, iyy=iyy, izz=izz,
            ixy=0, ixz=0, iyz=0
        )
        self.add_feature(mass_feature)
        
    def add_subcomponent(self, component: InteriorComponent, x_offset: float, y_offset: float, z_offset: float = 0):
        """
        Add a subcomponent (galley, bathroom, etc.) at a specified position within the service area
        
        Args:
            component: The component to add
            x_offset: Offset from the front of the service area
            y_offset: Offset from the left edge of the service area
            z_offset: Vertical offset from the floor
        """
        # Set component position relative to service area
        component.geometry.position.x = x_offset
        component.geometry.position.y = y_offset
        component.geometry.position.z = z_offset
        
        self.sub_components.append(component)
        self.add_child(component)
        
    def plot(self, color: str = 'lightgray', colors_dict: Dict[str, str] = None, plot_children: bool = True) -> Object3D:
        """Create a 3D visualization of the service area and its components"""
        obj = Object3D()
        
        # Plot the service area outline
        global_pos = self.get_global_position()
        service_box = create_box(
            width=self.geometry.parameters['width'],
            length=self.geometry.parameters['depth'],
            height=self.geometry.parameters['height']
        )
        # Convert vertices to float64 before adding
        service_box.vertices = service_box.vertices.astype(np.float64)
        service_box.vertices += global_pos
        service_box.metadata.update({
            'type': 'service_area',
            'color': color,
            'alpha': 0.3  # Make it semi-transparent
        })
        obj.add_shape(service_box)
        
        # Plot all sub-components
        if plot_children:
            for component in self.sub_components:
                component_color = colors_dict.get(component.__class__.__name__.lower(), 'gray') if colors_dict else 'gray'
                if hasattr(component, 'plot'):
                    if 'colors_dict' in component.plot.__code__.co_varnames:
                        component_obj = component.plot(color=component_color, colors_dict=colors_dict)
                    else:
                        component_obj = component.plot(color=component_color)
                obj.shapes.extend(component_obj.shapes)
        return obj

class ExitRow(InteriorComponent):
    """Emergency exit row configuration"""
    
    # Standard exit row measurements (in ft)
    EXIT_WIDTH = 20/12  # 20 inches minimum per FAA requirements
    EXIT_DEPTH = 30/12  # 30 inches for clear access
    EXIT_HEIGHT = 7  # 7 feet ceiling height
    
    def __init__(self, name: str, side: str = 'left'):
        """
        Initialize an exit row
        
        Args:
            name: Exit row name
            side: Which side of the aircraft ('left' or 'right')
        """
        super().__init__(name)
        self.side = side
        self.position = np.zeros(3)
        
        # Add mass analysis
        self.add_analysis(MassAnalysis())
        
        # Add mass for exit row structure (~200 lbs for door and reinforcement)
        mass = 200
        width = self.EXIT_WIDTH
        depth = self.EXIT_DEPTH
        height = self.EXIT_HEIGHT
        
        # Calculate moments of inertia
        ixx = (1/12) * mass * (height**2 + depth**2)
        iyy = (1/12) * mass * (width**2 + height**2)
        izz = (1/12) * mass * (width**2 + depth**2)
        
        # Add mass feature
        mass_feature = MassFeature(
            mass=mass,
            center_of_gravity=[depth/2, width/2, height/2],
            ixx=ixx, iyy=iyy, izz=izz,
            ixy=0, ixz=0, iyz=0
        )
        self.add_feature(mass_feature)
        
        self.geometry.parameters.update({
            'width': self.EXIT_WIDTH,
            'depth': self.EXIT_DEPTH,
            'height': self.EXIT_HEIGHT,
            'side': side
        })
    
    def plot(self, color: str = 'lightgreen') -> Object3D:
        """Create a 3D visualization of the exit row"""
        obj = Object3D()
        
        # Get global position
        global_pos = self.get_global_position()
        
        exit_space = create_box(
            width=self.EXIT_WIDTH,
            length=self.EXIT_DEPTH,
            height=self.EXIT_HEIGHT
        )
        exit_space.vertices += global_pos
        exit_space.metadata.update({
            'type': 'exit_row',
            'color': color
        })
        obj.add_shape(exit_space)
        return obj

class EconomyBlock(InteriorComponent):
    """A block of economy sections separated by exit rows"""
    
    def __init__(self, name: str, seat_groups: List[int], rows_per_section: List[int], 
                 seats_per_exit: int = 60, spacing: Dict[str, float] = None):
        """
        Initialize economy block with multiple sections and exit rows
        
        Args:
            name: Block name
            seat_groups: List of integers for seat configuration (e.g. [3,4,3])
            rows_per_section: List of integers specifying number of rows in each section
            seats_per_exit: Maximum number of seats per emergency exit (FAA requirement)
            spacing: Optional dictionary of spacing measurements to pass to EconomySections
        """
        super().__init__(name)
        self.sections: List[EconomySection] = []
        self.exit_rows: List[ExitRow] = []
        
        # Add mass analysis capability
        self.add_analysis(MassAnalysis())
        
        # Add structural mass for the block (~100 lbs per row for structure)
        total_rows = sum(rows_per_section)
        mass = 100 * total_rows
        
        # Calculate dimensions for moment of inertia
        width = sum(seat_groups) * (18/12)  # Convert inches to feet
        length = total_rows * (31/12)  # seat pitch in feet
        height = 7.0  # typical height in feet
        
        # Calculate moments of inertia for the structure
        ixx = (1/12) * mass * (height**2 + length**2)
        iyy = (1/12) * mass * (width**2 + height**2)
        izz = (1/12) * mass * (width**2 + length**2)
        
        # Add mass feature
        mass_feature = MassFeature(
            mass=mass,
            center_of_gravity=[length/2, width/2, height/2],
            ixx=ixx, iyy=iyy, izz=izz,
            ixy=0, ixz=0, iyz=0
        )
        self.add_feature(mass_feature)
        
        def add_exit_rows(name: str, current_x: float, section: EconomySection):
            # Left side exit
            left_exit = ExitRow(f"{name}_exit_left", side='left')
            left_exit.geometry.position.x = current_x
            left_exit.geometry.position.y = 0
            self.exit_rows.append(left_exit)
            self.add_child(left_exit)
            
            # Right side exit
            right_exit = ExitRow(f"{name}_exit_right", side='right')
            right_exit.geometry.position.x = current_x
            right_exit.geometry.position.y = section.total_width - right_exit.geometry.parameters['width']
            self.exit_rows.append(right_exit)
            self.add_child(right_exit)
            
            # Return new x position
            current_x += left_exit.geometry.parameters['depth']
            return current_x

        # Create sections and exit rows
        current_x = 0
        for i, num_rows in enumerate(rows_per_section):
            # Add economy section
            section = EconomySection(
                f"{name}_section_{i}",
                seat_groups=seat_groups,
                num_rows=num_rows,
                spacing=spacing
            )
            # add initial exit row
            if i == 0:
                current_x = add_exit_rows(f"{name}_section_{i}", current_x, section)
            
            section.geometry.position.x = current_x
            self.sections.append(section)
            self.add_child(section)
            
            current_x += section.total_length
            
            # Add exit rows after each section
            current_x = add_exit_rows(f"{name}_section_{i}", current_x, section)
        
        # Safety check: Ensure enough emergency exits
        total_exits = len(self.exit_rows)
        total_seats = sum(section.num_seats for section in self.sections)
        assert total_seats / total_exits <= seats_per_exit, \
            f"Safety violation: {total_seats} seats with only {total_exits} exits exceeds maximum of {seats_per_exit} seats per exit"

    @property
    def total_length(self) -> float:
        """Calculate total length of the economy block"""
        sections_length = sum(section.total_length for section in self.sections)
        exit_rows_length = sum(row.geometry.parameters['depth'] for row in self.exit_rows) / 2
        return sections_length + exit_rows_length

    @property
    def total_width(self) -> float:
        """Calculate total width of the economy block"""
        return max(section.total_width for section in self.sections)
    
    @property
    def num_seats(self) -> int:
        """Calculate total number of seats in the economy block"""
        return sum(section.num_seats for section in self.sections)

    def plot(self, *args, color: str = None, colors_dict: Dict[str, str] = None, **kwargs) -> Object3D:
        """Create a 3D visualization of the entire economy block"""
        obj = Object3D()
        
        # Plot all sections
        for section in self.sections:
            section_color = colors_dict.get('economysection', 'grey') if colors_dict else 'grey'
            section_obj = section.plot(color=section_color)
            obj.shapes.extend(section_obj.shapes)
            
        # Plot all exit rows
        for exit_row in self.exit_rows:
            exit_color = colors_dict.get('exitrow', 'red') if colors_dict else 'red'
            exit_obj = exit_row.plot(color=exit_color)
            obj.shapes.extend(exit_obj.shapes)
            
        return obj

class SingleFloorCabin(InteriorComponent):
    """Single floor cabin with economy sections and service areas"""
    
    def __init__(self, name: str):
        super().__init__(name)
        self.components: List[InteriorComponent] = []
        
        # Add mass analysis
        self.add_analysis(MassAnalysis())
        
        # Add mass for floor structure (~500 lbs base structure)
        mass = 500
        # Dimensions will be set later as components are added
        # Use initial estimates
        width = 20.0  # ft
        length = 100.0  # ft
        height = 7.0   # ft
        
        # Calculate moments of inertia
        ixx = (1/12) * mass * (height**2 + length**2)
        iyy = (1/12) * mass * (width**2 + height**2)
        izz = (1/12) * mass * (width**2 + length**2)
        
        # Add mass feature
        mass_feature = MassFeature(
            mass=mass,
            center_of_gravity=[length/2, width/2, height/2],
            ixx=ixx, iyy=iyy, izz=izz,
            ixy=0, ixz=0, iyz=0
        )
        self.add_feature(mass_feature)
        
    def add_component(self, component: InteriorComponent, x_position: float, y_position: float = 0.0) -> None:
        """Add a component at the specified position, centered around y=0"""
        if component.geometry is None:
            component.geometry = InteriorGeometry()
            
        # Get component width
        if hasattr(component, 'total_width'):
            width = component.total_width
        else:
            width = component.geometry.parameters.get('width', 0.0)
            
        # Center the component by offsetting by half its width
        centered_y = y_position - width/2
        
        # Preserve the component's z position
        current_z = component.geometry.position.z
        
        # Update position
        component.geometry.position.x = x_position
        component.geometry.position.y = centered_y
        # Only set z if it hasn't been set already
        if current_z == 0:
            component.geometry.position.z = 0.0
            
        self.components.append(component)
        self.add_child(component)

    @property
    def total_length(self) -> float:
        """Calculate total length of the cabin"""
        if not self.components:
            return 0.0
        return max(comp.geometry.position.x + getattr(comp, 'total_length', 
                  comp.geometry.parameters.get('depth', 0.0)) 
                  for comp in self.components)

    @property
    def total_width(self) -> float:
        """Calculate total width of the cabin"""
        if not self.components:
            return 0.0
        max_width = 0.0
        for comp in self.components:
            comp_width = getattr(comp, 'total_width', comp.geometry.parameters.get('width', 0.0))
            max_width = max(max_width, comp_width)
        return max_width

    def plot(self, color: str = None, colors_dict: Dict[str, str] = None) -> Object3D:
        """Create a 3D visualization of the entire cabin"""
        obj = Object3D()
        
        # Plot all components with their global positions
        for component in self.components:
            # Determine color based on component type
            component_color = color
            if colors_dict and component.__class__.__name__.lower() in colors_dict:
                component_color = colors_dict[component.__class__.__name__.lower()]
            
            # Pass colors_dict to child components that support it
            if hasattr(component, 'plot') and 'colors_dict' in component.plot.__code__.co_varnames:
                component_obj = component.plot(color=component_color, colors_dict=colors_dict)
            else:
                component_obj = component.plot(color=component_color)
            
            obj.shapes.extend(component_obj.shapes)
            
        return obj

   
    def add_mass(self, mass: float, cg_x: float = None, cg_y: float = None, cg_z: float = None):
        """Add mass properties to the component
        
        Args:
            mass: Mass in kg
            cg_x, cg_y, cg_z: Optional CG coordinates. If None, uses component center
        """
        # If CG not specified, use component center
        if cg_x is None or cg_y is None or cg_z is None:
            # Estimate component center based on geometry and position
            if hasattr(self, 'total_length'):
                length = self.total_length
            else:
                length = self.geometry.parameters.get('depth', 0.0)
                
            if hasattr(self, 'total_width'):
                width = self.total_width
            else:
                width = self.geometry.parameters.get('width', 0.0)
                
            height = self.geometry.parameters.get('height', 0.0)
            
            # Default CG is at component center
            if cg_x is None:
                cg_x = self.geometry.position.x + length / 2
            if cg_y is None:
                cg_y = self.geometry.position.y + width / 2
            if cg_z is None:
                cg_z = self.geometry.position.z + height / 2
        
        # Simple moment of inertia calculation (treating as point mass)
        # For a more accurate model, this would need to be improved
        ixx = mass * ((cg_y - self.geometry.position.y)**2 + (cg_z - self.geometry.position.z)**2)
        iyy = mass * ((cg_x - self.geometry.position.x)**2 + (cg_z - self.geometry.position.z)**2)
        izz = mass * ((cg_x - self.geometry.position.x)**2 + (cg_y - self.geometry.position.y)**2)
        ixy = -mass * (cg_x - self.geometry.position.x) * (cg_y - self.geometry.position.y)
        ixz = -mass * (cg_x - self.geometry.position.x) * (cg_z - self.geometry.position.z)
        iyz = -mass * (cg_y - self.geometry.position.y) * (cg_z - self.geometry.position.z)
        
        # Create and add the mass feature
        mass_feature = MassFeature(
            mass=mass,
            center_of_gravity=[cg_x, cg_y, cg_z],
            ixx=ixx, iyy=iyy, izz=izz,
            ixy=ixy, ixz=ixz, iyz=iyz
        )
        self.add_feature(mass_feature)
        
        return mass_feature
    
    def get_cg(self):
        """Calculate center of gravity based on mass analysis results
        
        Returns:
            tuple: (cg_x, cg_y, cg_z) or None if no mass analysis results
        """
        # Run mass analysis if not already done
        if 'mass_analysis' not in self.analysis_results:
            self.run_analysis('mass_analysis')
            
        results = self.analysis_results.get('mass_analysis', {})
        if 'total_mass' in results and results['total_mass'] > 0:
            cg_x = results.get('cg_x', self.geometry.position.x)
            cg_y = results.get('cg_y', self.geometry.position.y)
            cg_z = results.get('cg_z', self.geometry.position.z)
            return (cg_x, cg_y, cg_z)
        return None

def main():
    """Create and plot a three-view of an example cabin layout"""
    import matplotlib.pyplot as plt
    import os
    
    # Create the single floor cabin
    cabin = SingleFloorCabin("main_cabin")
    
    # Create front economy block
    front_economy = EconomyBlock(
        name="front_economy",
        seat_groups=[3, 4, 3],
        rows_per_section=[4,4],
        seats_per_exit=60
    )
    
    # Create service areas with galleys and bathrooms
    GALLEY_SPACING = 2.0  # Spacing after service areas
    
    # Front service area
    front_service = ServiceComponent("front_service", width=front_economy.total_width, depth=4.0)
    front_galley = Galley("front_galley", width=10.0, depth=3.0, height=7.0)
    front_bathroom = Bathroom("front_bathroom", width=3.0, depth=3.0)
    
    # Position components inside service area
    front_service.add_subcomponent(front_galley, 0.5, front_economy.total_width/2 - 5.0)
    front_service.add_subcomponent(front_bathroom, 0.5, 2.0)
    
    # Add components to cabin in sequence
    current_x = 0.0
    
    # Front service area
    cabin.add_component(front_service, current_x)
    current_x += front_service.geometry.parameters['depth'] + GALLEY_SPACING
    
    # Front economy block
    cabin.add_component(front_economy, current_x)
    current_x += front_economy.total_length + GALLEY_SPACING
    
    # Print cabin statistics and debug info
    print(f"\nCabin Statistics:")
    print(f"Total Length: {cabin.total_length:.1f} ft")
    print(f"Total Width: {cabin.total_width:.1f} ft")
    print(f"Regular Economy Section Width: {front_economy.total_width:.1f} ft")
    
    plot = True
    if plot:
        # Create top view only
        fig, ax_top = plt.subplots(figsize=(12, 8))
        
        # Get the 3D object
        cabin_obj = cabin.plot()
        
        # Plot top view (x-y plane)
        for shape in cabin_obj.shapes:
            vertices = shape.vertices
            x, y = vertices[:, 0], vertices[:, 1]
            color = shape.metadata.get('color', 'blue')
            alpha = shape.metadata.get('alpha', 0.7)
            ax_top.fill(x, y, color=color, alpha=alpha)
        
        ax_top.set_aspect('equal')
        ax_top.set_title('Cabin Layout - Top View', fontsize=14)
        ax_top.set_xlabel('Length (ft)')
        ax_top.set_ylabel('Width (ft)')
        
        # Ensure the output directory exists
        os.makedirs('assets', exist_ok=True)
        
        # Save the figure
        plt.savefig('assets/test_cabin_layout.png')
        plt.close()

if __name__ == "__main__":
    main() 