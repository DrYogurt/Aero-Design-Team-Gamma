from typing import List, Dict
import numpy as np
from aircraft_design.components.interior.base import InteriorComponent
from aircraft_design.components.interior.seating import SeatingSection, SeatGeometry, Seat
from aircraft_design.components.interior.aisle import Aisle
from aircraft_design.components.interior.service import Galley, Bathroom
from aircraft_design.core.plotting import Object3D, Shape3D, create_box, plot_orthographic_views, plot_3d_object

class EconomySection(SeatingSection):
    """Standard economy class section configuration"""
    
    # Standard economy class measurements (in ft)
    SEAT_WIDTH = 18/12  # 18 inches
    SEAT_HEIGHT = 39/12  # 39 inches
    SEAT_SPACING = 31/12  # 31 inches (center-to-center distance)
    AISLE_WIDTH = 20/12  # 20 inches
    SEAT_DEPTH = 18/12  # 18 inches

    def __init__(self, name: str, seat_groups: List[int], num_rows: int = 1):
        """
        Initialize economy class section
        
        Args:
            name: Section name
            seat_groups: List of integers representing number of seats in each group
                        (e.g. [3,4,3] for a 3-4-3 configuration)
            num_rows: Number of rows in the section
        """
        super().__init__(name, section_type='economy')
        self.num_rows = num_rows
        self.seat_groups_config = seat_groups
        self._configure_rows()
    
    @property
    def num_seats(self) -> int:
        """Calculate total number of seats in the section"""
        return sum(self.seat_groups_config) * self.num_rows

    @property   
    def total_length(self) -> float:
        """Calculate total length of the section"""
        return self.num_rows * self.SEAT_SPACING

    @property
    def total_width(self) -> float:
        """Calculate total width of the section including aisles"""
        total_width = 0
        for i, num_seats in enumerate(self.seat_groups_config):
            total_width += num_seats * self.SEAT_WIDTH
            if i < len(self.seat_groups_config) - 1:
                total_width += self.AISLE_WIDTH
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
                    seat.geometry.parameters.update({
                        'width': self.SEAT_WIDTH,
                        'depth': self.SEAT_DEPTH,
                        'height': self.SEAT_HEIGHT,
                        'seat_spacing': self.SEAT_SPACING
                    })
                    
                    # Set seat position
                    seat.geometry.position.x = row * self.SEAT_SPACING
                    seat.geometry.position.y = current_y + seat_idx * self.SEAT_WIDTH
                    seat.geometry.position.z = 0
                    
                    # Set seat metadata
                    seat.row = row
                    seat.group = group_idx
                    
                    seats_in_group.append(seat)
                    self.add_child(seat)
            
            self.seat_groups.append(seats_in_group)
            
            # Add aisle after each group except the last one
            if group_idx < len(self.seat_groups_config) - 1:
                current_y += num_seats * self.SEAT_WIDTH + self.AISLE_WIDTH
                self.add_aisle(self.AISLE_WIDTH, group_idx)
            else:
                current_y += num_seats * self.SEAT_WIDTH

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

class ExitRow(InteriorComponent):
    """Emergency exit row configuration"""
    
    # Standard exit row measurements (in ft)
    EXIT_WIDTH = 20/12  # 20 inches minimum per FAA requirements
    EXIT_DEPTH = 30/12  # 30 inches for clear access
    EXIT_HEIGHT = 7  # 7 feet ceiling height
    
    def __init__(self, name: str):
        super().__init__(name)
        self.position = np.zeros(3)  # Initialize position vector
        self.geometry.parameters.update({
            'width': self.EXIT_WIDTH,
            'depth': self.EXIT_DEPTH,
            'height': self.EXIT_HEIGHT
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
                 seats_per_exit: int = 60):
        """
        Initialize economy block with multiple sections and exit rows
        
        Args:
            name: Block name
            seat_groups: List of integers for seat configuration (e.g. [3,4,3])
            rows_per_section: List of integers specifying number of rows in each section
            seats_per_exit: Maximum number of seats per emergency exit (FAA requirement)
        """
        super().__init__(name)
        self.sections: List[EconomySection] = []
        self.exit_rows: List[ExitRow] = []
        
        def add_exit_row(name: str, current_x: float, section: EconomySection):
            exit_row = ExitRow(f"{name}_exit_left")
            exit_row.geometry.position.x = current_x
            exit_row.geometry.position.y = 0
            self.exit_rows.append(exit_row)
            self.add_child(exit_row)
            exit_row = ExitRow(f"{name}_exit_right")
            exit_row.geometry.position.x = current_x
            exit_row.geometry.position.y = section.total_width
            current_x += exit_row.geometry.parameters['depth']
            return current_x

        # Create sections and exit rows
        current_x = 0
        for i, num_rows in enumerate(rows_per_section):
            # Add economy section
            section = EconomySection(
                f"{name}_section_{i}",
                seat_groups=seat_groups,
                num_rows=num_rows
            )
            # add initial exit row
            if i == 0:
                current_x = add_exit_row(f"{name}_section_{i}", current_x, section)
            
            section.geometry.position.x = current_x
            self.sections.append(section)
            self.add_child(section)
            
            current_x += section.total_length
            
            # Add exit rows after each section
            current_x = add_exit_row(f"{name}_section_{i}", current_x, section)
        # Safety check: Ensure enough emergency exits
        total_exits = len(self.exit_rows) + 2  # Add 2 for the exits at start and end of block
        total_seats = sum(section.num_seats for section in self.sections)
        assert total_seats / total_exits <= seats_per_exit, \
            f"Safety violation: {total_seats} seats with only {total_exits} exits exceeds maximum of {seats_per_exit} seats per exit"

    @property
    def total_length(self) -> float:
        """Calculate total length of the economy block"""
        sections_length = sum(section.total_length for section in self.sections)
        exit_rows_length = sum(row.geometry.parameters['depth'] for row in self.exit_rows)
        return sections_length + exit_rows_length

    @property
    def total_width(self) -> float:
        """Calculate total width of the economy block"""
        return max(section.total_width for section in self.sections)
    
    @property
    def num_seats(self) -> int:
        """Calculate total number of seats in the economy block"""
        return sum(section.num_seats for section in self.sections)

    def plot(self, color: str = None, colors_dict: Dict[str, str] = None) -> Object3D:
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

def main():
    """Create and plot a three-view of an example cabin layout"""
    import matplotlib.pyplot as plt
    
    # Create the single floor cabin
    cabin = SingleFloorCabin("main_cabin")
    
    # Create front economy block
    front_economy = EconomyBlock(
        name="front_economy",
        seat_groups=[3, 4, 3],
        rows_per_section=[4,4],
        num_exit_rows=2,
        seats_per_exit=60
    )
    
    # Create rear economy block
    middle_economy = EconomyBlock(
        name="middle_economy",
        seat_groups=[3, 6, 3],
        rows_per_section=[4,5,4],
        num_exit_rows=2
    )

    # Create rear economy block
    rear_economy = EconomyBlock(
        name="rear_economy",
        seat_groups=[3,6,3],
        rows_per_section=[6,8],
        num_exit_rows=2,
        seats_per_exit=60
    )
    
    # Create galleys and bathrooms with adjusted dimensions
    GALLEY_SPACING = 2.0  # Add 2 feet of space after each galley
    mid_galley = Galley("mid_galley", width=10.0, depth=4.0, height=7.0)
    rear_galley = Galley("rear_galley", width=15.0, depth=4.0, height=7.0)
    
    bathroom_width = 3.0
    bathroom_depth = 3.0
    mid_bathroom = Bathroom("mid_bathroom", width=bathroom_width, depth=bathroom_depth)
    back_bathroom = Bathroom("back_bathroom", width=bathroom_width, depth=bathroom_depth)
    
    # Add components to cabin in sequence
    current_x = 0.0
    
    # Front economy block
    cabin.add_component(front_economy, current_x)
    current_x += front_economy.total_length + GALLEY_SPACING  # Add spacing before galley
    
    # Mid galley and bathroom
    cabin.add_component(mid_galley, current_x)
    cabin.add_component(mid_bathroom, current_x)  # Will be automatically centered
    current_x += mid_galley.geometry.parameters['depth'] + GALLEY_SPACING  # Add spacing after galley
    
    # Middle economy block
    cabin.add_component(middle_economy, current_x)
    current_x += middle_economy.total_length + GALLEY_SPACING  # Add spacing before rear galley
    
    # Rear economy block
    cabin.add_component(rear_economy, current_x)
    current_x += rear_economy.total_length + GALLEY_SPACING  # Add spacing before rear galley
    
    # Rear galley and bathroom
    cabin.add_component(rear_galley, current_x)
    cabin.add_component(back_bathroom, current_x)  # Will be automatically centered
   
    # Print cabin statistics and debug info
    print(f"\nCabin Statistics:")
    print(f"Total Length: {cabin.total_length:.1f} ft")
    print(f"Total Width: {cabin.total_width:.1f} ft")
    print(f"Total Seats: {front_economy.num_seats + middle_economy.num_seats + rear_economy.num_seats}")
    
    if False:
        for component in cabin.components:
            print(f"\n{component.name}:")
            print(f"Position: ({component.geometry.position.x:.1f}, {component.geometry.position.y:.1f}, {component.geometry.position.z:.1f})")
            print(f"Parameters: {component.geometry.parameters}")
            print(f"Absolute Position: {component.get_global_position()}")
            if isinstance(component, EconomyBlock):
                for section in component.sections:
                    print(f"  {section.name} absolute pos: {section.get_global_position()}")
                for exit_row in component.exit_rows:
                    print(f"  {exit_row.name} absolute pos: {exit_row.get_global_position()}")
    plot = True
    if plot:
        # Create orthographic views
        fig, (ax_top, ax_side, ax_front) = plot_orthographic_views(cabin.plot())
        
        # save the top view only
        # Add title
        fig.suptitle('Single Floor Cabin Layout', fontsize=16)
    
        plt.savefig('assets/blended_wing_cabin_second_floor.png')
        plt.close()
if __name__ == "__main__":
    main() 