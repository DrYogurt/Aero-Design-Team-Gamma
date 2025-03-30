from aircraft_design.components.interior.cabin import SingleFloorCabin, EconomyBlock, ServiceComponent
from aircraft_design.components.interior.service import Galley, Bathroom, Stairs, InteriorComponent
import copy
from aircraft_design.analysis.mass_analysis import MassAnalysis, MassFeature
import matplotlib.pyplot as plt
import numpy as np
from aircraft_design.components.interior.cabin import plot_orthographic_views
from aircraft_design.core.plotting import plot_top_view, plot_side_view, plot_front_view, plot_cg


######
# bottom cabin
######


first_floor_sections = [

    EconomyBlock(
    name="floor_1_section_1",
    seat_groups=[3, 6, 3],  # 3-6-3 configuration
    rows_per_section=[9,9],  # 30 rows total
    seats_per_exit=55
    ),
    EconomyBlock(
    name="floor_1_section_2",
    seat_groups=[3, 6, 3],  # 3-6-3 configuration
    rows_per_section=[9, 9],  # 30 rows total
    seats_per_exit=55
    ),
    EconomyBlock(
    name="floor_1_section_3",
    seat_groups=[3, 6, 3],  # 3-6-3 configuration
    rows_per_section=[6,8],  # 30 rows total
    seats_per_exit=55
    )
]

######
# top cabin
######

second_floor_sections = [
    EconomyBlock(
    name="floor_2_section_1",
    seat_groups=[3, 6, 3],  # 3-6-3 configuration
    rows_per_section=[9,9],  # 30 rows total
    seats_per_exit=55
    ),
    EconomyBlock(
    name="floor_2_section_2",
    seat_groups=[3, 6, 3],  # 3-6-3 configuration
    rows_per_section=[9,9,5],  # 30 rows total
    seats_per_exit=55
    )
]



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

total_width = first_floor_sections[0].total_width
aisle_width = total_width - 2*galley_dimensions["width"]

# Add components to the big service area
big_service = ServiceComponent("big_service", width=total_width, depth=galley_dimensions["depth"] + 2*bathroom_dimensions["depth"], height=7)
bathroom1 = Bathroom("bathroom1", **bathroom_dimensions)
bathroom2 = Bathroom("bathroom2", **bathroom_dimensions)
bathroom3 = Bathroom("bathroom3", **bathroom_dimensions)
bathroom4 = Bathroom("bathroom4", **bathroom_dimensions)
galley1 = Galley("galley1", **galley_dimensions)
galley2 = Galley("galley2", **galley_dimensions)
stairs = Stairs("stairs", **stairs_dimensions)

# Position components inside big service area
big_service.add_subcomponent(bathroom1, 0,0)
big_service.add_subcomponent(galley1, bathroom_dimensions["depth"], 0)
big_service.add_subcomponent(bathroom2, bathroom_dimensions["depth"] + galley_dimensions["depth"], 0)

big_service.add_subcomponent(bathroom3, 0, bathroom_dimensions["width"] + aisle_width)
big_service.add_subcomponent(galley2, bathroom_dimensions["depth"], bathroom_dimensions["width"] + aisle_width)
big_service.add_subcomponent(bathroom4, bathroom_dimensions["depth"] + galley_dimensions["depth"], bathroom_dimensions["width"] + aisle_width)
big_service.add_subcomponent(stairs, bathroom_dimensions["depth"] + galley_dimensions["depth"], total_width/2 - stairs_dimensions["width"]/2)


# Create a horizontal service area with swapped width/depth compared to big service area

# swap width and depth
bathroom_dimensions["width"], bathroom_dimensions["depth"] = bathroom_dimensions["depth"], bathroom_dimensions["width"]
galley_dimensions["width"], galley_dimensions["depth"] = galley_dimensions["depth"], galley_dimensions["width"]

small_service = ServiceComponent("small_service", 
                               width=total_width, 
                               depth=galley_dimensions["depth"], 
                               height=7)
# Add components to the small service area
small_bathroom1 = Bathroom("small_bathroom1", **bathroom_dimensions)
small_bathroom2 = Bathroom("small_bathroom2", **bathroom_dimensions)
small_galley = Galley("small_galley", **galley_dimensions)

# Position components horizontally in a row (bathroom-galley-bathroom)
# Place them in the middle of the aisle
# Calculate the horizontal center position
center_y = small_service.geometry.parameters['width'] / 2
total_component_width = bathroom_dimensions["width"] + galley_dimensions["width"] + bathroom_dimensions["width"]
start_y = center_y - total_component_width/2
# Position components from the bottom left corner
small_service.add_subcomponent(small_bathroom1, 0, start_y)
small_service.add_subcomponent(small_galley, 0, start_y + bathroom_dimensions["width"])
small_service.add_subcomponent(small_bathroom2, 0, start_y + bathroom_dimensions["width"] + galley_dimensions["width"])

# Create the first floor layout
first_floor = InteriorComponent("first_floor")
# Add front service area
front_service = big_service
first_floor.add_child(front_service)
front_service.geometry.position.x = 0
current_position = front_service.geometry.parameters['depth']

# Add cabin sections with small service areas between them
for i, section in enumerate(first_floor_sections):
    # Add the cabin section
    first_floor.add_child(section)
    section.geometry.position.x = current_position
    current_position += section.total_length
    
    # Add small service area after each section except the last one
    if i < len(first_floor_sections) - 1:
        mid_service = copy.deepcopy(small_service)
        mid_service.name = f"mid_service_{i+1}"
        first_floor.add_child(mid_service)
        mid_service.geometry.position.x = current_position
        current_position += mid_service.geometry.parameters['depth']

# Add back service area at the end
back_service = copy.deepcopy(big_service)
back_service.name = "back_service"
first_floor.add_child(back_service)
back_service.geometry.position.x = current_position

# Create the second floor layout
second_floor = InteriorComponent("second_floor")

# Add front service area
front_service_2 = copy.deepcopy(big_service)
front_service_2.name = "front_service_2"
second_floor.add_child(front_service_2)
front_service_2.geometry.position.x = 0
current_position = front_service_2.geometry.parameters['depth']

# Add cabin sections with small service areas between them
for i, section in enumerate(second_floor_sections):
    # Add the cabin section
    second_floor.add_child(section)
    section.geometry.position.x = current_position
    current_position += section.total_length
    
    # Add small service area after each section except the last one
    if i < len(second_floor_sections) - 1:
        mid_service = copy.deepcopy(small_service)
        mid_service.name = f"mid_service_2_{i+1}"
        second_floor.add_child(mid_service)
        mid_service.geometry.position.x = current_position
        current_position += mid_service.geometry.parameters['depth']

# Add back service area at the end
back_service_2 = copy.deepcopy(big_service)
back_service_2.name = "back_service_2"
second_floor.add_child(back_service_2)
back_service_2.geometry.position.x = current_position

# Set vertical position for second floor
second_floor.geometry.position.z = 7  # Assuming 7 feet height for first floor

# Create a single cabin interior component containing both floors
cabin_interior = InteriorComponent("cabin_interior")
cabin_interior.add_child(first_floor)
cabin_interior.add_child(second_floor)

# Add mass analysis to cabin interior, but no mass feature
cabin_interior.add_analysis(MassAnalysis())

if __name__ == "__main__":
    # Since we've fixed the mass_analysis.py, we can directly run it on the cabin_interior
    # This will automatically account for the positions of child components
    cabin_interior.run_analysis('mass_analysis', analyze_children=True)
    
    # Get results directly from cabin_interior
    cabin_results = cabin_interior.analysis_results['mass_analysis']
    
    # Calculate cabin dimensions directly by iterating over components
    cabin_length = 0
    cabin_width = 0
    
    # Get dimensions from first floor
    for component in first_floor.children:
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
                
            cabin_length = max(cabin_length, end_x)
            
            # Calculate width
            width = 0
            if hasattr(component, 'total_width'):
                if callable(getattr(component, 'total_width')):
                    width = component.total_width()
                else:
                    width = component.total_width
            elif 'width' in component.geometry.parameters:
                width = component.geometry.parameters['width']
                
            cabin_width = max(cabin_width, width)
    
    # Do the same for second floor
    for component in second_floor.children:
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
                
            cabin_length = max(cabin_length, end_x)
            
            # Calculate width
            width = 0
            if hasattr(component, 'total_width'):
                if callable(getattr(component, 'total_width')):
                    width = component.total_width()
                else:
                    width = component.total_width
            elif 'width' in component.geometry.parameters:
                width = component.geometry.parameters['width']
                
            cabin_width = max(cabin_width, width)
    
    cabin_height = second_floor.geometry.position.z + 7.0
    
    # Now we can get the CG directly from the cabin_interior analysis
    total_mass = cabin_results['total_mass']
    overall_cg_x = cabin_results['cg_x']
    overall_cg_y = cabin_results['cg_y']
    overall_cg_z = cabin_results['cg_z']
    
    # For comparison, also get the individual floor CGs
    first_floor.run_analysis('mass_analysis', analyze_children=True)
    second_floor.run_analysis('mass_analysis', analyze_children=True)
    first_floor_results = first_floor.analysis_results['mass_analysis']
    second_floor_results = second_floor.analysis_results['mass_analysis']
    
    # Print dimensions for debugging
    print(f"\n=== Cabin Dimensions ===")
    print(f"Length: {cabin_length:.2f} ft")
    print(f"Width: {cabin_width:.2f} ft")
    print(f"Height: {cabin_height:.2f} ft")
    
    # Print overall results
    print("\n=== Overall Cabin Mass Analysis Results ===")
    print(f"Total Cabin Mass: {total_mass:.1f} lbs")
    print(f"Cabin CG: ({overall_cg_x:.2f}, {overall_cg_y:.2f}, {overall_cg_z:.2f})")
    
    # Print individual floor results
    print("\n=== Individual Floor Results ===")
    print(f"First Floor Mass: {first_floor_results['total_mass']:.1f} lbs")
    print(f"First Floor CG: ({first_floor_results['cg_x']:.2f}, {first_floor_results['cg_y']:.2f}, {first_floor_results['cg_z']:.2f})")
    print(f"Second Floor Mass: {second_floor_results['total_mass']:.1f} lbs")
    print(f"Second Floor CG: ({second_floor_results['cg_x']:.2f}, {second_floor_results['cg_y']:.2f}, {second_floor_results['cg_z']:.2f})")
    
    # Basic rectangular prism moments of inertia (use the total_mass from analysis)
    cabin_Ixx = total_mass * (cabin_width**2 + cabin_height**2) / 12
    cabin_Iyy = total_mass * (cabin_length**2 + cabin_height**2) / 12
    cabin_Izz = total_mass * (cabin_length**2 + cabin_width**2) / 12
    
    print("\n=== Overall Moments of Inertia ===")
    print(f"Ixx: {cabin_Ixx:.2f} lbs-ft²")
    print(f"Iyy: {cabin_Iyy:.2f} lbs-ft²")
    print(f"Izz: {cabin_Izz:.2f} lbs-ft²")
    
    # Create a figure for the overall cabin view with CG
    fig = plt.figure(figsize=(15, 10))
    ax = fig.add_subplot(111)
    
    # Plot both floors
    obj_first = first_floor.plot()
    _, ax = plot_top_view(obj_first, fig=fig, ax=ax)
    
    # Plot second floor with different color
    obj_second = second_floor.plot()
    for shape in obj_second.shapes:
        shape.metadata['color'] = 'red'  # Make second floor red for distinction
        shape.metadata['alpha'] = 0.4    # More translucent
    
    _, ax = plot_top_view(obj_second, fig=fig, ax=ax)
    
    # Add overall CG marker
    plot_cg(ax, overall_cg_x, overall_cg_y, color='green', markersize=15, label='Overall CG')
    
    # Add individual floor CG markers
    plot_cg(ax, first_floor_results['cg_x'], first_floor_results['cg_y'], 
            color='blue', markersize=8, label='First Floor CG')
    plot_cg(ax, second_floor_results['cg_x'], second_floor_results['cg_y'], 
            color='red', markersize=8, label='Second Floor CG')
    
    # Add text annotation
    annotation_text = (
        f"Total Mass: {total_mass:.0f} lbs\n"
        f"Overall CG: ({overall_cg_x:.2f}, {overall_cg_y:.2f}, {overall_cg_z:.2f})\n"
        f"Cabin Dimensions: {cabin_length:.1f} x {cabin_width:.1f} x {cabin_height:.1f} ft\n"
        f"Ixx: {cabin_Ixx:.0f} lbs-ft²  Iyy: {cabin_Iyy:.0f} lbs-ft²  Izz: {cabin_Izz:.0f} lbs-ft²"
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