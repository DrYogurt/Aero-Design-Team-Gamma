from typing import Dict, Any
import numpy as np
import matplotlib.pyplot as plt

from aircraft_design.core.base import Component, Position
from aircraft_design.analysis.mass_analysis import MassAnalysis, MassFeature
from aircraft_design.core.plotting import (
    Object3D,
    plot_cg,
    plot_top_view,
    plot_side_view,
    plot_front_view,
    plot_orthographic_views,
)

from aircraft_design.final_design.final_wing import Wing
from aircraft_design.final_design.final_cabin import Cabin
from aircraft_design.final_design.final_fuselage import Fuselage
from aircraft_design.final_design.final_tails import HorizontalTail, VerticalTail
from aircraft_design.components.propulsion.engine import Engine
from aircraft_design.components.propulsion.fuel_tanks import FuelTank
from aircraft_design.final_design.final_landing_gear import LandingGear

class Aircraft(Component):
    """Main aircraft component that combines wing, cabin, and fuselage"""
    
    def __init__(self):
        """Initialize the aircraft with its major components"""
        super().__init__(name="aircraft")
        
        # Create fuselage
        self.fuselage = Fuselage()
        self.add_child(self.fuselage)
        
        # Create wing
        self.wing = Wing(
            wing_tip_position = 85
        )
        self.add_child(self.wing)
        
        # Create cabin
        self.cabin = Cabin()
        
        self.cabin.cabin_interior.geometry.position = Position(
            x=self.fuselage.nose_length + 10,  # Position cabin after nose section
            y=-self.cabin.cabin_width / 2,  # Position cabin at the centerline of the aircraft
            z=9  # Position cabin at the center of the tall section height
        )
        self.add_child(self.cabin)
        
        # Create horizontal tail
        htail_position = Position(
            x=self.fuselage.nose_length + self.fuselage.short_fuselage_length + self.fuselage.tail_length - 30,
            y=0,
            z=self.fuselage.short_fuselage_height - 5
        )
        self.horizontal_tail = HorizontalTail(
            position=htail_position,
            wing_ref=self.wing,
        )
        # set fuel tank on horizontal tail
        for child in self.horizontal_tail.children:
            if isinstance(child, FuelTank):
                child.set_fill_level(0.8)
        self.add_child(self.horizontal_tail)
        
        # Create vertical tail
        vtail_position = Position(
            x=self.fuselage.nose_length + self.fuselage.short_fuselage_length + self.fuselage.tail_length - 35,
            y=0,
            z=self.fuselage.short_fuselage_height
        )
        self.vertical_tail = VerticalTail(
            position=vtail_position,
            wing_ref=self.wing,
        )
        self.add_child(self.vertical_tail)
        
        # Create four engines
        # Engine parameters
        engine_mass = 20000  # lbs
        engine_radius = 14.0/2  # ft
        engine_length = 19.0  # ft
        engine_thrust = 26710  # lbs

        self.engines = []
        # Calculate engine positions (two on each wing)
        wing_span = self.wing.geometry.parameters['span']
        engine_y_positions = [
            wing_span/6,  # Inner left engine
            wing_span/3,  # Outer left engine
            -wing_span/6,  # Inner right engine
            -wing_span/3   # Outer right engine
        ]

        # Create and position engines
        for i, y_pos in enumerate(engine_y_positions):
            engine = Engine(f"engine_{i+1}", engine_thrust)
            
            # Configure engine geometry
            engine.geometry.parameters.update({
                'radius': engine_radius,
                'length': engine_length,
            })
            
            
            # Position engine relative to the wing
            # Calculate engine position accounting for wing sweep and dihedral
            wing_sweep_rad = np.radians(self.wing.geometry.parameters.get('le_sweep', 25))
            wing_dihedral_rad = np.radians(self.wing.geometry.parameters.get('dihedral', 5))
            
            # Calculate x-offset due to sweep (moves engine aft as y increases)
            x_sweep_offset = abs(y_pos) * np.tan(wing_sweep_rad)
            #print(f"x_sweep_offset: {x_sweep_offset}")
            # Calculate z-offset due to dihedral (raises engine as y increases)
            z_dihedral_offset = abs(y_pos) * np.tan(wing_dihedral_rad)
            #print(f"z_dihedral_offset: {z_dihedral_offset}")
            
            
            
            engine.geometry.position = Position(
                x= -5 + x_sweep_offset,  # Position engines relative to wing leading edge, adjusted for sweep
                y=y_pos,  # Position on wing
                z=z_dihedral_offset  # Position adjusted for wing dihedral
            )
            engine.position = engine.geometry.position
            
            # Add mass feature to engine
            engine.add_feature(MassFeature(
                mass=engine_mass,
                center_of_gravity=[engine_length/2, engine_radius, engine_radius],  # CG at engine center
                ixx=engine_mass * (engine_radius**2 + engine_length**2/4) / 4,  # Simplified inertia calculations
                iyy=engine_mass * (engine_radius**2 + engine_length**2/4) / 4,
                izz=engine_mass * engine_radius**2 / 2
            ))
            
            # Add mass analysis to engine
            engine.add_analysis(MassAnalysis())
            
            self.wing.add_child(engine)
            self.engines.append(engine)

        # Add landing gears
        # 1. Nose landing gear (4 wheels) at 30ft from nose
        nose_gear = LandingGear(
            name="nose_landing_gear",
            position=Position(x=30, y=0, z=0),
            num_wheels=4,
            wheel_mass=2222
        )
        self.add_child(nose_gear)

        # 2. Main landing gears at 133ft from nose
        # Left side: two sets of 6 wheels
        main_gear_left_1 = LandingGear(
            name="main_landing_gear_left_1",
            position=Position(x=133, y=15, z=0),  # 15ft to the left
            num_wheels=6,
            wheel_mass=2222
        )
        self.add_child(main_gear_left_1)

        main_gear_left_2 = LandingGear(
            name="main_landing_gear_left_2",
            position=Position(x=133, y=25, z=0),  # 25ft to the left
            num_wheels=6,
            wheel_mass=2222
        )
        self.add_child(main_gear_left_2)

        # Right side: two sets of 4 wheels
        main_gear_right_1 = LandingGear(
            name="main_landing_gear_right_1",
            position=Position(x=133, y=-15, z=0),  # 15ft to the right
            num_wheels=4
        )
        self.add_child(main_gear_right_1)

        main_gear_right_2 = LandingGear(
            name="main_landing_gear_right_2",
            position=Position(x=133, y=-25, z=0),  # 25ft to the right
            num_wheels=4,
            wheel_mass=2222
        )
        self.add_child(main_gear_right_2)

        # 3. Rear landing gears at 155ft from nose
        # Left side: one set of 6 wheels
        rear_gear_left = LandingGear(
            name="rear_landing_gear_left",
            position=Position(x=155, y=20, z=0),  # 20ft to the left
            num_wheels=6,
            wheel_mass=2222
        )
        self.add_child(rear_gear_left)

        # Right side: one set of 6 wheels
        rear_gear_right = LandingGear(
            name="rear_landing_gear_right",
            position=Position(x=155, y=-20, z=0),  # 20ft to the right
            num_wheels=6,
            wheel_mass=2222
        )
        self.add_child(rear_gear_right)

        # Add mass analysis
        self.add_analysis(MassAnalysis())
    
    def get_mass_properties(self):
        """Get mass properties of the complete aircraft"""
        # Run mass analysis on the entire aircraft
        self.run_analysis('mass_analysis', analyze_children=True)
        aircraft_results = self.analysis_results['mass_analysis']
        
        return {
            'total_mass': aircraft_results['total_mass'],
            'cg_x': aircraft_results['cg_x'],
            'cg_y': aircraft_results['cg_y'],
            'cg_z': aircraft_results['cg_z'],
            'ixx': aircraft_results['total_ixx'],
            'iyy': aircraft_results['total_iyy'],
            'izz': aircraft_results['total_izz']
        }
    
    def plot(self, *args, **kwargs) -> Object3D:
        """Create a visualization object for the complete aircraft"""
        # Use the built-in recursive plotting system from Component
        return super().plot(colors_dict={
            'fuselage': 'blue',
            'wing': 'lightgray',
            'cabin': 'lightblue',
            'horizontal_tail': 'lightgray',
            'vertical_tail': 'lightgray',
            'engine': 'purple'
        })

    def to_dict(self) -> Dict[str, Any]:
        """
        Convert aircraft configuration to a dictionary for serialization.
        
        Returns:
            Dict[str, Any]: Dictionary representation of the aircraft
        """
        aircraft_dict = {
            'name': self.name,
            'wing_position': self.wing.geometry.position.x,
            'htail_span': self.horizontal_tail.geometry.parameters['span'],
            'htail_area': self.horizontal_tail.geometry.area,
            'vtail_height': self.vertical_tail.geometry.parameters['height'],
            'vtail_area': self.vertical_tail.geometry.area,
        }
        
        # Include mass properties if they've been calculated
        if hasattr(self, 'analysis_results') and 'mass_analysis' in self.analysis_results:
            results = self.analysis_results['mass_analysis']
            aircraft_dict['mass_properties'] = {
                'total_mass': results['total_mass'],
                'cg_x': results['cg_x'],
                'cg_y': results['cg_y'],
                'cg_z': results['cg_z'],
                'ixx': results['total_ixx'],
                'iyy': results['total_iyy'],
                'izz': results['total_izz']
            }
        
        return aircraft_dict


if __name__ == "__main__":
    # Create an aircraft instance
    aircraft = Aircraft()
    
    # Now test with full tanks
    print("\n=== Aircraft Properties with Full Tanks ===")
    
    # Get mass properties with full tanks
    mass_props_full = aircraft.get_mass_properties()
    print(f"Total Mass: {mass_props_full['total_mass']:.1f} lbs")
    print(f"CG Position: ({mass_props_full['cg_x']:.2f}, {mass_props_full['cg_y']:.2f}, {mass_props_full['cg_z']:.2f}) ft")
    

    # First test with drained tanks
    print("\n=== Aircraft Properties with Drained Tanks ===")
    # empty all fuel tanks
    for child in aircraft.wing.children:
        if isinstance(child, FuelTank):
            child.set_fill_level(0.0)
    # Empty horizontal tail tanks
    for child in aircraft.horizontal_tail.children:
        if isinstance(child, FuelTank):
            child.set_fill_level(0.0)
    for child in aircraft.vertical_tail.children:
        if isinstance(child, FuelTank):
            child.set_fill_level(0.0)
    
    # Get mass properties with drained tanks
    mass_props_drained = aircraft.get_mass_properties()
    print(f"Total Mass: {mass_props_drained['total_mass']:.1f} lbs")
    print(f"CG Position: ({mass_props_drained['cg_x']:.2f}, {mass_props_drained['cg_y']:.2f}, {mass_props_drained['cg_z']:.2f}) ft")
    print(f"Fuel Mass: {mass_props_full['total_mass'] - mass_props_drained['total_mass']:.1f} lbs")



    # Print moments of inertia (using drained configuration for consistency)
    print("\n=== Moments of Inertia ===")
    print(f"  Ixx: {mass_props_drained['ixx']:.0f} lbs-ft²")
    print(f"  Iyy: {mass_props_drained['iyy']:.0f} lbs-ft²")
    print(f"  Izz: {mass_props_drained['izz']:.0f} lbs-ft²")

    plot_aircraft = True
    if plot_aircraft:
        # Create figures for orthographic views (using drained configuration)
        aircraft_obj = aircraft.plot()
        print(f"aircraft objects created")

        # Plot all orthographic views
        fig = plt.figure(figsize=(18, 12))
        fig, (ax_top, ax_side, ax_front) = plot_orthographic_views(aircraft_obj, fig=fig)
        
        # Add overall CG marker to top view
        plot_cg(ax_top, mass_props_drained['cg_x'], mass_props_drained['cg_y'], 
                color='red', markersize=15, label='Aircraft CG')
        
        # Add title and legend
        plt.suptitle("Complete Aircraft - Orthographic Views", fontsize=16)
        ax_top.legend(loc='upper right')
        
        # Save orthographic views
        plt.tight_layout()
        plt.savefig('assets/aircraft_orthographic_views.png')
        plt.close()
        
        print(f"orthographic views created")
        # Create individual views with CG markers
        # Top view
        fig_top = plt.figure(figsize=(15, 10))
        ax = fig_top.add_subplot(111)
        _, ax = plot_top_view(aircraft_obj, fig=fig_top, ax=ax)
        plot_cg(ax, mass_props_drained['cg_x'], 0, 
                color='red', markersize=15, label='Aircraft CG')
        
        # Add text annotation
        """
        annotation_text = (
            f"Total Mass: {mass_props['total_mass']:.0f} lbs\n"
            f"CG: ({mass_props['cg_x']:.2f}, {mass_props['cg_y']:.2f}, {mass_props['cg_z']:.2f}) ft\n"
            f"Ixx: {mass_props['ixx']:.0f} lbs-ft²  Iyy: {mass_props['iyy']:.0f} lbs-ft²  Izz: {mass_props['izz']:.0f} lbs-ft²"
        )
        
        ax.annotate(annotation_text, 
                xy=(0.05, 0.05), xycoords='axes fraction',
                bbox=dict(boxstyle="round,pad=0.3", fc="white", ec="black", alpha=0.8))
        """
        ax.set_title("Aircraft - Top View")
        ax.set_aspect('equal')
        ax.legend(loc='upper right')
        
        plt.tight_layout()
        plt.savefig('assets/aircraft_top_view.png')
        plt.close()
        print(f"top view created")
        # Side view
        fig_side = plt.figure(figsize=(15, 6))
        ax = fig_side.add_subplot(111)
        _, ax = plot_side_view(aircraft_obj, fig=fig_side, ax=ax)
        plot_cg(ax, mass_props_drained['cg_x'], mass_props_drained['cg_z'], 
                color='red', markersize=15, label='Aircraft CG')
        
        ax.set_title("Aircraft - Side View")
        ax.set_aspect('equal')
        ax.legend(loc='upper right')
        
        plt.tight_layout()
        plt.savefig('assets/aircraft_side_view.png')
        plt.close()
        print(f"side view created")
        # Front view
        fig_front = plt.figure(figsize=(10, 10))
        ax = fig_front.add_subplot(111)
        _, ax = plot_front_view(aircraft_obj, fig=fig_front, ax=ax)
        plot_cg(ax, 0, mass_props_drained['cg_z'], 
                color='red', markersize=15, label='Aircraft CG')
        
        ax.set_title("Aircraft - Front View")
        ax.set_aspect('equal')
        ax.legend(loc='upper right')    
        plt.tight_layout()
        plt.savefig('assets/aircraft_front_view.png')
        plt.close()
        print(f"front view created")
        print("Aircraft visualizations saved to assets directory.")
