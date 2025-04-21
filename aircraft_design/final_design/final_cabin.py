from typing import List, Dict, Any
import numpy as np
import matplotlib.pyplot as plt
import copy

from aircraft_design.core.base import Component, Position
from aircraft_design.components.interior.cabin import SingleFloorCabin, EconomyBlock, ServiceComponent
from aircraft_design.components.interior.service import Galley, Bathroom, Stairs, InteriorComponent
from aircraft_design.analysis.mass_analysis import MassAnalysis, MassFeature
from aircraft_design.core.plotting import plot_top_view, plot_cg, Object3D, plot_front_view


class Cabin(Component):
    """Main cabin component with double-deck interior layout"""
    
    def __init__(self, position: Position = None):
        """Initialize the cabin component with its interior layout"""
        super().__init__(name="cabin")
        
        self.position = position if position else Position()
        # Initialize cabin parameters
        self.top_floor_height = 7.0  # feet
        self.bottom_floor_height = 7.0  # feet
        self.cabin_height = self.top_floor_height + self.bottom_floor_height
        
        # Create interior components
        self._create_top_floor()
        self._create_bottom_floor()
        
        # Add floors to cabin interior
        self.cabin_interior = InteriorComponent("cabin_interior")
        self.cabin_interior.add_child(self.top_floor)
        self.cabin_interior.add_child(self.bottom_floor)
        self.add_child(self.cabin_interior)

        self._compute_dimensions()
        
        # Add mass analysis
        self._add_mass_analysis()
    
    def _create_service_areas(self):
        """Create service area components (bathrooms, galleys)"""
        # Define dimensions
        bathroom_dimensions = {
            "width": 3.0,
            "depth": 3.0,
            "height": 7.0
        }
        galley_dimensions = {
            "width": 3.0,
            "depth": 8.0,
            "height": 7.0
        }
        stairs_dimensions = {
            "width": 3.0,
            "depth": 3.0,
            "height": 7.0
        }
        
        # Get total width from economy block
        seat_groups = [3, 6, 3]  # 3-6-3 configuration
        seat_width = 1.5  # feet per seat
        aisle_width = 1.5  # feet per aisle
        total_width = sum(seat_groups) * seat_width + (len(seat_groups) - 1) * aisle_width
        
        aisle_width = total_width - 2*galley_dimensions["width"]
        
        # Create big service area
        big_service = ServiceComponent("big_service", 
                                      width=total_width, 
                                      depth=galley_dimensions["depth"] + 2*bathroom_dimensions["depth"],
                                      height=7)
        
        # Create service components
        bathroom1 = Bathroom("bathroom1", **bathroom_dimensions)
        bathroom2 = Bathroom("bathroom2", **bathroom_dimensions)
        bathroom3 = Bathroom("bathroom3", **bathroom_dimensions)
        bathroom4 = Bathroom("bathroom4", **bathroom_dimensions)
        galley1 = Galley("galley1", **galley_dimensions)
        galley2 = Galley("galley2", **galley_dimensions)
        stairs = Stairs("stairs", **stairs_dimensions)
        
        # Position components inside big service area
        big_service.add_subcomponent(bathroom1, 0, 0)
        big_service.add_subcomponent(galley1, bathroom_dimensions["depth"], 0)
        big_service.add_subcomponent(bathroom2, bathroom_dimensions["depth"] + galley_dimensions["depth"], 0)
        
        big_service.add_subcomponent(bathroom3, 0, bathroom_dimensions["width"] + aisle_width)
        big_service.add_subcomponent(galley2, bathroom_dimensions["depth"], bathroom_dimensions["width"] + aisle_width)
        big_service.add_subcomponent(bathroom4, bathroom_dimensions["depth"] + galley_dimensions["depth"], bathroom_dimensions["width"] + aisle_width)
        big_service.add_subcomponent(stairs, bathroom_dimensions["depth"] + galley_dimensions["depth"], total_width/2 - stairs_dimensions["width"]/2)
        
        # Create small service area (swapped dimensions)
        small_bathroom_dimensions = bathroom_dimensions.copy()
        small_galley_dimensions = galley_dimensions.copy()
        
        # Swap width and depth
        small_bathroom_dimensions["width"], small_bathroom_dimensions["depth"] = small_bathroom_dimensions["depth"], small_bathroom_dimensions["width"]
        small_galley_dimensions["width"], small_galley_dimensions["depth"] = small_galley_dimensions["depth"], small_galley_dimensions["width"]
        
        small_service = ServiceComponent("small_service", 
                                       width=total_width, 
                                       depth=small_galley_dimensions["depth"], 
                                       height=7)
                                       
        # Add components to the small service area
        small_bathroom1 = Bathroom("small_bathroom1", **small_bathroom_dimensions)
        small_bathroom2 = Bathroom("small_bathroom2", **small_bathroom_dimensions)
        small_galley = Galley("small_galley", **small_galley_dimensions)
        
        # Position components horizontally in a row
        center_y = small_service.geometry.parameters['width'] / 2
        total_component_width = small_bathroom_dimensions["width"] + small_galley_dimensions["width"] + small_bathroom_dimensions["width"]
        start_y = center_y - total_component_width/2
        
        small_service.add_subcomponent(small_bathroom1, 0, start_y)
        small_service.add_subcomponent(small_galley, 0, start_y + small_bathroom_dimensions["width"])
        small_service.add_subcomponent(small_bathroom2, 0, start_y + small_bathroom_dimensions["width"] + small_galley_dimensions["width"])
        
        self.big_service = big_service
        self.small_service = small_service
        self.total_width = total_width
    
    def _create_top_floor(self):
        """Create the top floor layout with economy sections and service areas"""
        # Create service areas
        self._create_service_areas()
        
        # Define economy sections
        top_floor_sections = [
            EconomyBlock(
                name="floor_1_section_1",
                seat_groups=[3, 6, 3],  # 3-6-3 configuration
                rows_per_section=[6, 9],  # 18 rows total
                seats_per_exit=55
            ),
            EconomyBlock(
                name="floor_1_section_2",
                seat_groups=[3, 6, 3],  # 3-6-3 configuration
                rows_per_section=[9, 9],  # 18 rows total
                seats_per_exit=55
            ),
            EconomyBlock(
                name="floor_1_section_3",
                seat_groups=[3, 6, 3],  # 3-6-3 configuration
                rows_per_section=[9, 9],  # 18 rows total
                seats_per_exit=55
            )
        ]
        
        # Create the top floor layout
        self.top_floor = InteriorComponent("top_floor")
        self.top_floor.geometry.position = self.position + Position(z=self.bottom_floor_height + 2)
        
        # Add front service area
        front_service = self.big_service
        self.top_floor.add_child(front_service)
        front_service.geometry.position.x = 0
        current_position = front_service.geometry.parameters['depth']
        
        # Add cabin sections with small service areas between them
        for i, section in enumerate(top_floor_sections):
            # Add the cabin section
            self.top_floor.add_child(section)
            section.geometry.position.x = current_position
            current_position += section.total_length
            
            # Add small service area after each section except the last one
            if i < len(top_floor_sections) - 1:
                mid_service = copy.deepcopy(self.small_service)
                mid_service.name = f"mid_service_{i+1}"
                self.top_floor.add_child(mid_service)
                mid_service.geometry.position.x = current_position
                current_position += mid_service.geometry.parameters['depth']
        
        # Add back service area at the end
        back_service = copy.deepcopy(self.big_service)
        back_service.name = "back_service"
        self.top_floor.add_child(back_service)
        back_service.geometry.position.x = current_position
        
        # Store sections for later use
        self.top_floor_sections = top_floor_sections
    
    def _create_bottom_floor(self):
        """Create the bottom floor layout with economy sections and service areas"""
        # Define economy sections
        bottom_floor_sections = [
            EconomyBlock(
                name="floor_2_section_1",
                seat_groups=[3, 6, 3],  # 3-6-3 configuration
                rows_per_section=[9, 9],  # 18 rows total
                seats_per_exit=55
            ),
            EconomyBlock(
                name="floor_2_section_2",
                seat_groups=[3, 6, 3],  # 3-6-3 configuration
                rows_per_section=[9, 9, 5],  # 23 rows total
                seats_per_exit=55
            )
        ]
        
        # Create the bottom floor layout
        self.bottom_floor = InteriorComponent("bottom_floor")
        self.bottom_floor.geometry.position = self.position
        # Add front service area
        front_service_2 = copy.deepcopy(self.big_service)
        front_service_2.name = "front_service_2"
        self.bottom_floor.add_child(front_service_2)
        front_service_2.geometry.position.x = 0
        current_position = front_service_2.geometry.parameters['depth']
        
        # Add cabin sections with small service areas between them
        for i, section in enumerate(bottom_floor_sections):
            # Add the cabin section
            self.bottom_floor.add_child(section)
            section.geometry.position.x = current_position
            current_position += section.total_length
            
            # Add small service area after each section except the last one
            if i < len(bottom_floor_sections) - 1:
                mid_service = copy.deepcopy(self.small_service)
                mid_service.name = f"mid_service_2_{i+1}"
                self.bottom_floor.add_child(mid_service)
                mid_service.geometry.position.x = current_position
                current_position += mid_service.geometry.parameters['depth']
        
        # Add back service area at the end
        back_service_2 = copy.deepcopy(self.big_service)
        back_service_2.name = "back_service_2"
        self.bottom_floor.add_child(back_service_2)
        back_service_2.geometry.position.x = current_position
        
        # Store sections for later use
        self.bottom_floor_sections = bottom_floor_sections
    
    def _compute_dimensions(self):
        """Compute the dimensions of the cabin"""
        self.cabin_length = 0
        self.cabin_width = 0
        
        # Get dimensions from top floor
        for component in self.top_floor.children:
            if hasattr(component, 'geometry'):
                # Calculate the end position of this component
                end_x = component.geometry.position.x
                if hasattr(component, 'total_length'):
                    # If it has a total_length method/property
                    if callable(getattr(component, 'total_length')):
                        end_x += component.total_length()
                    else:
                        end_x += component.total_length
                elif 'depth' in component.geometry.parameters:
                    end_x += component.geometry.parameters['depth']
                    
                self.cabin_length = max(self.cabin_length, end_x)
                
                # Calculate width
                width = 0
                if hasattr(component, 'total_width'):
                    if callable(getattr(component, 'total_width')):
                        width = component.total_width()
                    else:
                        width = component.total_width
                elif 'width' in component.geometry.parameters:
                    width = component.geometry.parameters['width']
                    
                self.cabin_width = max(self.cabin_width, width)
        
        # Do the same for bottom floor
        for component in self.bottom_floor.children:
            if hasattr(component, 'geometry'):
                # Calculate the end position of this component
                end_x = component.geometry.position.x
                if hasattr(component, 'total_length'):
                    # If it has a total_length method/property
                    if callable(getattr(component, 'total_length')):
                        end_x += component.total_length()
                    else:
                        end_x += component.total_length
                elif 'depth' in component.geometry.parameters:
                    end_x += component.geometry.parameters['depth']
                    
                self.cabin_length = max(self.cabin_length, end_x)
                
                # Calculate width
                width = 0
                if hasattr(component, 'total_width'):
                    if callable(getattr(component, 'total_width')):
                        width = component.total_width()
                    else:
                        width = component.total_width
                elif 'width' in component.geometry.parameters:
                    width = component.geometry.parameters['width']
                    
                self.cabin_width = max(self.cabin_width, width)
    
    def _add_mass_analysis(self):
        """Add mass analysis to the cabin"""
        self.cabin_interior.add_analysis(MassAnalysis())
        self.add_analysis(MassAnalysis())
        
        # Calculate seat count
        self.total_seats = 0
        for section in self.top_floor_sections + self.bottom_floor_sections:
            self.total_seats += section.num_seats
    
    def calculate_moments_of_inertia(self):
        """Calculate moments of inertia based on cabin dimensions and mass"""
        # Run mass analysis
        self.cabin_interior.run_analysis('mass_analysis', analyze_children=True)
        cabin_results = self.cabin_interior.analysis_results['mass_analysis']
        total_mass = cabin_results['total_mass']
        
        # Basic rectangular prism moments of inertia
        self.ixx = total_mass * (self.cabin_width**2 + self.cabin_height**2) / 12
        self.iyy = total_mass * (self.cabin_length**2 + self.cabin_height**2) / 12
        self.izz = total_mass * (self.cabin_length**2 + self.cabin_width**2) / 12
        
        return {
            'ixx': self.ixx,
            'iyy': self.iyy,
            'izz': self.izz
        }
    
    def get_mass_properties(self):
        """Get mass properties of the cabin"""
        # Run mass analysis
        self.cabin_interior.run_analysis('mass_analysis', analyze_children=True)
        cabin_results = self.cabin_interior.analysis_results['mass_analysis']
        
        return {
            'total_mass': cabin_results['total_mass'],
            'cg_x': cabin_results['cg_x'],
            'cg_y': cabin_results['cg_y'],
            'cg_z': cabin_results['cg_z']
        }
    
    def plot(self, *args, **kwargs) -> Object3D:
        """Create a 3D visualization of the cabin"""
        # Create a combined object from both floors
        cabin_obj = self.top_floor.plot()
        
        # Add bottom floor with different color
        bottom_floor_obj = self.bottom_floor.plot()
        for shape in bottom_floor_obj.shapes:
            shape.metadata['color'] = 'red'  # Make bottom floor red for distinction
            shape.metadata['alpha'] = 0.4    # More translucent
        
        # Combine objects
        cabin_obj.shapes.extend(bottom_floor_obj.shapes)
        cabin_obj.metadata['name'] = 'cabin'
        
        return cabin_obj


if __name__ == "__main__":
    # Create a cabin instance
    cabin = Cabin()
    
    """
    # Print emergency exit positions
    print("\n=== Emergency Exit Positions ===")
    print("Top Floor Exits:")
    for section in cabin.top_floor_sections:
        for exit_row in section.exit_rows:
            global_pos = exit_row.get_global_position()
            print(f"{exit_row.name}: ({global_pos[0]:.2f}, {global_pos[1]:.2f}, {global_pos[2]:.2f}) ft")
    
    print("\Bottom Floor Exits:")
    for section in cabin.bottom_floor_sections:
        for exit_row in section.exit_rows:
            global_pos = exit_row.get_global_position()
            print(f"{exit_row.name}: ({global_pos[0]:.2f}, {global_pos[1]:.2f}, {global_pos[2]:.2f}) ft")
    """
    # Calculate mass properties and moments of inertia
    mass_props = cabin.get_mass_properties()
    inertia = cabin.calculate_moments_of_inertia()
    
    # Get individual floor properties for comparison
    cabin.top_floor.run_analysis('mass_analysis', analyze_children=True)
    cabin.bottom_floor.run_analysis('mass_analysis', analyze_children=True)
    top_floor_results = cabin.top_floor.analysis_results['mass_analysis']
    bottom_floor_results = cabin.bottom_floor.analysis_results['mass_analysis']
    
    # Print dimensions
    print(f"\n=== Cabin Dimensions ===")
    print(f"Length: {cabin.cabin_length:.2f} ft")
    print(f"Width: {cabin.cabin_width:.2f} ft")
    print(f"Height: {cabin.cabin_height:.2f} ft")
    print(f"Total Seats: {cabin.total_seats}")
    
    # Print overall results
    print("\n=== Overall Cabin Mass Analysis Results ===")
    print(f"Total Cabin Mass: {mass_props['total_mass']:.1f} lbs")
    print(f"Cabin CG: ({mass_props['cg_x']:.2f}, {mass_props['cg_y']:.2f}, {mass_props['cg_z']:.2f})")
    
    # Print individual floor results
    print("\n=== Individual Floor Results ===")
    print(f"top Floor Mass: {top_floor_results['total_mass']:.1f} lbs")
    print(f"Top Floor CG: ({top_floor_results['cg_x']:.2f}, {top_floor_results['cg_y']:.2f}, {top_floor_results['cg_z']:.2f})")
    print(f"Bottom Floor Mass: {bottom_floor_results['total_mass']:.1f} lbs")
    print(f"Bottom Floor CG: ({bottom_floor_results['cg_x']:.2f}, {bottom_floor_results['cg_y']:.2f}, {bottom_floor_results['cg_z']:.2f})")
    
    # Print moments of inertia
    print("\n=== Overall Moments of Inertia ===")
    print(f"Ixx: {inertia['ixx']:.2f} lbs-ft²")
    print(f"Iyy: {inertia['iyy']:.2f} lbs-ft²")
    print(f"Izz: {inertia['izz']:.2f} lbs-ft²")
    
    # Create a figure for the overall cabin view with CG
    fig = plt.figure(figsize=(15, 10))
    ax = fig.add_subplot(111)
    
    # Plot cabin
    cabin_visualization = cabin.plot()
    _, ax = plot_top_view(cabin_visualization, fig=fig, ax=ax)
    
    # Add overall CG marker
    plot_cg(ax, mass_props['cg_x'], mass_props['cg_y'], color='green', markersize=15, label='Overall CG')
    
    # Add individual floor CG markers
    plot_cg(ax, top_floor_results['cg_x'], top_floor_results['cg_y'], 
            color='blue', markersize=8, label='top Floor CG')
    plot_cg(ax, bottom_floor_results['cg_x'], bottom_floor_results['cg_y'], 
            color='red', markersize=8, label='bottom Floor CG')
    
    # Add text annotation
    annotation_text = (
        f"Total Mass: {mass_props['total_mass']:.0f} lbs\n"
        f"Overall CG: ({mass_props['cg_x']:.2f}, {mass_props['cg_y']:.2f}, {mass_props['cg_z']:.2f})\n"
        f"Cabin Dimensions: {cabin.cabin_length:.1f} x {cabin.cabin_width:.1f} x {cabin.cabin_height:.1f} ft\n"
        f"Ixx: {inertia['ixx']:.0f} lbs-ft²  Iyy: {inertia['iyy']:.0f} lbs-ft²  Izz: {inertia['izz']:.0f} lbs-ft²"
    )
    
    ax.annotate(annotation_text, 
               xy=(0.05, 0.05), xycoords='axes fraction',
               bbox=dict(boxstyle="round,pad=0.3", fc="white", ec="black", alpha=0.8))
                       
    ax.set_title("Complete Aircraft Cabin - Top View with Overall CG")
    ax.set_aspect('equal')
    ax.legend(loc='upper right')
    
    plt.tight_layout()
    plt.savefig('assets/overall_cabin_cg.png')
    plt.close('all')

    # plot the front view of the cabin
    fig = plt.figure(figsize=(15, 10))
    ax = fig.add_subplot(111)
    cabin_visualization = cabin.plot()
    _, ax = plot_front_view(cabin_visualization, fig=fig, ax=ax)
    plt.savefig('assets/cabin_front_view.png')
    plt.close('all')