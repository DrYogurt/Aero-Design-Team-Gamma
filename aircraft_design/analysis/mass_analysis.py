from typing import Dict, Any
from aircraft_design.core.base import AnalysisModule, Feature, Component

class MassFeature(Feature):
    """Feature that defines mass properties for a component"""
    def __init__(self, mass: float = 0.0, center_of_gravity: float = 0.0):
        super().__init__(name="mass")
        self.parameters = {
            "mass": mass,  # Mass in kg
            "center_of_gravity": center_of_gravity  # Center of gravity in meters
        }

    def modify_analysis(self, analysis: AnalysisModule, component: 'Component') -> None:
        """Add mass to the analysis if it's a MassAnalysis"""
        if isinstance(analysis, MassAnalysis):
            analysis.parameters["component_mass"] = self.parameters["mass"]

    def validate(self) -> bool:
        """Validate that mass is non-negative"""
        return self.parameters["mass"] >= 0

class MassAnalysis(AnalysisModule):
    """Analysis module for calculating total mass of a component and its children"""
    def __init__(self):
        super().__init__(name="mass_analysis")
        self.parameters = {
            "component_mass": 0.0  # Mass of the component itself
        }
        self.required_parameters = ["component_mass"]

    def run(self, component: Component) -> Dict[str, Any]:
        """Calculate total mass by summing component mass and all children masses"""
        # Get mass of current component
        total_mass = self.parameters["component_mass"]
        
        # Add masses of all children
        for child in component.children:
            if "mass_analysis" in child.analyses:
                child_results = child.run_analysis("mass_analysis")
                total_mass += child_results["total_mass"]

        self.results = {
            "total_mass": total_mass,
            "component_mass": self.parameters["component_mass"]
        }
        
        return self.results 