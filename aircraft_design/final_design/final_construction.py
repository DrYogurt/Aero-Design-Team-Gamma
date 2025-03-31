from typing import List, Dict, Any
import numpy as np
import matplotlib.pyplot as plt

from aircraft_design.core.base import Component, Position
from aircraft_design.analysis.mass_analysis import MassAnalysis
from aircraft_design.core.plotting import plot_orthographic_views, plot_cg, Object3D, plot_top_view, plot_side_view, plot_front_view

# Import components from other files
from aircraft_design.final_design.final_wing import Wing
from aircraft_design.final_design.final_cabin import Cabin
from aircraft_design.final_design.final_fuselage import Fuselage


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
            nose_length=self.fuselage.nose_length,
            tall_fuselage_length=self.fuselage.tall_fuselage_length
        )
        self.add_child(self.wing)
        
        # Create cabin
        self.cabin = Cabin(Position(
            x=self.fuselage.nose_length + 10,  # Position cabin after nose section
            y=0,  # Position cabin at the centerline of the aircraft
            z=self.fuselage.tall_fuselage_height / 2  # Position cabin at the center of the tall section height
        ))
        #self.add_child(self.cabin)
        
        # Add mass analysis (but no mass feature since this is just a container)
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
            'wing': 'gray',
            'cabin': 'lightblue'
        })


if __name__ == "__main__":
    # Create an aircraft instance
    aircraft = Aircraft()
    
    # Calculate mass properties
    mass_props = aircraft.get_mass_properties()
    
    # Print aircraft properties
    print(f"\n=== Aircraft Properties ===")
    print(f"Total Mass: {mass_props['total_mass']:.1f} lbs")
    print(f"CG Position: ({mass_props['cg_x']:.2f}, {mass_props['cg_y']:.2f}, {mass_props['cg_z']:.2f}) ft")
    print(f"Moments of Inertia:")
    print(f"  Ixx: {mass_props['ixx']:.0f} lbs-ft²")
    print(f"  Iyy: {mass_props['iyy']:.0f} lbs-ft²")
    print(f"  Izz: {mass_props['izz']:.0f} lbs-ft²")
    
    # Create figures for orthographic views
    aircraft_obj = aircraft.plot()
    
    # Plot all orthographic views
    fig = plt.figure(figsize=(18, 12))
    fig, (ax_top, ax_side, ax_front) = plot_orthographic_views(aircraft_obj, fig=fig)
    
    # Add overall CG marker to top view
    plot_cg(ax_top, mass_props['cg_x'], mass_props['cg_y'], 
            color='red', markersize=15, label='Aircraft CG')
    
    # Add title and legend
    plt.suptitle("Complete Aircraft - Orthographic Views", fontsize=16)
    ax_top.legend(loc='upper right')
    
    # Save orthographic views
    plt.tight_layout()
    plt.savefig('assets/aircraft_orthographic_views.png')
    plt.close()
    
    # Create individual views with CG markers
    # Top view
    fig_top = plt.figure(figsize=(15, 10))
    ax = fig_top.add_subplot(111)
    _, ax = plot_top_view(aircraft_obj, fig=fig_top, ax=ax)
    plot_cg(ax, mass_props['cg_x'], mass_props['cg_y'], 
            color='red', markersize=15, label='Aircraft CG')
    
    # Add text annotation
    annotation_text = (
        f"Total Mass: {mass_props['total_mass']:.0f} lbs\n"
        f"CG: ({mass_props['cg_x']:.2f}, {mass_props['cg_y']:.2f}, {mass_props['cg_z']:.2f}) ft\n"
        f"Ixx: {mass_props['ixx']:.0f} lbs-ft²  Iyy: {mass_props['iyy']:.0f} lbs-ft²  Izz: {mass_props['izz']:.0f} lbs-ft²"
    )
    
    ax.annotate(annotation_text, 
               xy=(0.05, 0.05), xycoords='axes fraction',
               bbox=dict(boxstyle="round,pad=0.3", fc="white", ec="black", alpha=0.8))
    
    ax.set_title("Aircraft - Top View")
    ax.set_aspect('equal')
    ax.legend(loc='upper right')
    
    plt.tight_layout()
    plt.savefig('assets/aircraft_top_view.png')
    plt.close()
    
    # Side view
    fig_side = plt.figure(figsize=(15, 6))
    ax = fig_side.add_subplot(111)
    _, ax = plot_side_view(aircraft_obj, fig=fig_side, ax=ax)
    plot_cg(ax, mass_props['cg_x'], mass_props['cg_z'], 
            color='red', markersize=15, label='Aircraft CG')
    
    ax.set_title("Aircraft - Side View")
    ax.set_aspect('equal')
    ax.legend(loc='upper right')
    
    plt.tight_layout()
    plt.savefig('assets/aircraft_side_view.png')
    plt.close()
    
    # Front view
    fig_front = plt.figure(figsize=(10, 10))
    ax = fig_front.add_subplot(111)
    _, ax = plot_front_view(aircraft_obj, fig=fig_front, ax=ax)
    plot_cg(ax, mass_props['cg_y'], mass_props['cg_z'], 
            color='red', markersize=15, label='Aircraft CG')
    
    ax.set_title("Aircraft - Front View")
    ax.set_aspect('equal')
    ax.legend(loc='upper right')
    
    plt.tight_layout()
    plt.savefig('assets/aircraft_front_view.png')
    plt.close()
    
    print("Aircraft visualizations saved to assets directory.")
