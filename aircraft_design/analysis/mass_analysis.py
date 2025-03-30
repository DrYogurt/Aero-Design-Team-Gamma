from typing import Dict, Any, List
from aircraft_design.core.base import AnalysisModule, Feature, Component

class MassFeature(Feature):
    """Feature that defines mass properties for a component"""
    def __init__(self, mass: float = 0.0, 
                 center_of_gravity: List[float] = None,  # Changed from float to List[float]
                 ixx: float = 0.0, iyy: float = 0.0, izz: float = 0.0,
                 ixy: float = 0.0, ixz: float = 0.0, iyz: float = 0.0):
        super().__init__(name="mass")
        if center_of_gravity is None:
            center_of_gravity = [0.0, 0.0, 0.0]  # Default CG at origin
            
        self.parameters = {
            "mass": mass,  # Mass in kg
            "center_of_gravity": center_of_gravity,  # Center of gravity as [x,y,z]
            "ixx": ixx,  # Moment of inertia about x-axis in kg*m^2
            "iyy": iyy,  # Moment of inertia about y-axis in kg*m^2
            "izz": izz,  # Moment of inertia about z-axis in kg*m^2
            "ixy": ixy,  # Product of inertia xy in kg*m^2
            "ixz": ixz,  # Product of inertia xz in kg*m^2
            "iyz": iyz   # Product of inertia yz in kg*m^2
        }

    def modify_analysis(self, analysis: AnalysisModule, component: 'Component') -> None:
        """Add mass and inertia properties to the analysis if it's a MassAnalysis"""
        if isinstance(analysis, MassAnalysis):
            analysis.parameters.update({
                "component_mass": self.parameters["mass"],
                "component_cg_x": self.parameters["center_of_gravity"][0],  # Added CG components
                "component_cg_y": self.parameters["center_of_gravity"][1],
                "component_cg_z": self.parameters["center_of_gravity"][2],
                "component_ixx": self.parameters["ixx"],
                "component_iyy": self.parameters["iyy"],
                "component_izz": self.parameters["izz"],
                "component_ixy": self.parameters["ixy"],
                "component_ixz": self.parameters["ixz"],
                "component_iyz": self.parameters["iyz"]
            })

    def validate(self) -> bool:
        """Validate that mass and moments of inertia are non-negative"""
        return (self.parameters["mass"] >= 0 and
                self.parameters["ixx"] >= 0 and
                self.parameters["iyy"] >= 0 and
                self.parameters["izz"] >= 0 and
                isinstance(self.parameters["center_of_gravity"], (list, tuple)) and
                len(self.parameters["center_of_gravity"]) == 3)  # Added CG validation

class MassAnalysis(AnalysisModule):
    """Analysis module for calculating total mass and moments of inertia"""
    def __init__(self):
        super().__init__(name="mass_analysis")
        self.parameters = {
            "component_mass": 0.0,
            "component_ixx": 0.0,
            "component_iyy": 0.0,
            "component_izz": 0.0,
            "component_ixy": 0.0,
            "component_ixz": 0.0,
            "component_iyz": 0.0,
            "cg_x": 0.0,
            "cg_y": 0.0,
            "cg_z": 0.0
        }

    def run(self, component: 'Component') -> Dict[str, Any]:
        """Calculate total mass properties by summing component and all children"""
        # Initialize totals
        total_mass = 0.0
        total_ixx = 0.0
        total_iyy = 0.0
        total_izz = 0.0
        total_ixy = 0.0
        total_ixz = 0.0
        total_iyz = 0.0
        mass_moment_x = 0.0
        mass_moment_y = 0.0
        mass_moment_z = 0.0

        # Get component's own mass properties from its mass feature
        for feature in component.features:
            if isinstance(feature, MassFeature):
                mass = feature.parameters["mass"]
                cg = feature.parameters["center_of_gravity"]
                total_mass += mass
                
                # Use the actual CG coordinates (not squared)
                mass_moment_x += mass * cg[0]
                mass_moment_y += mass * cg[1]
                mass_moment_z += mass * cg[2]
                
                total_ixx += feature.parameters["ixx"]
                total_iyy += feature.parameters["iyy"]
                total_izz += feature.parameters["izz"]
                total_ixy += feature.parameters.get("ixy", 0.0)
                total_ixz += feature.parameters.get("ixz", 0.0)
                total_iyz += feature.parameters.get("iyz", 0.0)

        # Add mass properties from all children
        for child in component.children:
            if "mass_analysis" in child.analyses:
                try:
                    #child_results = child.run_analysis("mass_analysis")
                    child_results = child.analyses["mass_analysis"].results
                    if child_results:  # Make sure we got valid results
                        child_mass = child_results["total_mass"]
                        
                        # Get the child component's position relative to this component
                        rel_pos_x = child.geometry.position.x if hasattr(child.geometry, 'position') else 0
                        rel_pos_y = child.geometry.position.y if hasattr(child.geometry, 'position') else 0
                        rel_pos_z = child.geometry.position.z if hasattr(child.geometry, 'position') else 0
                        
                        # Add the relative position to the child's CG to get absolute position
                        child_cg_x = child_results["cg_x"] + rel_pos_x
                        child_cg_y = child_results["cg_y"] + rel_pos_y
                        child_cg_z = child_results["cg_z"] + rel_pos_z
                        
                        total_mass += child_mass
                        mass_moment_x += child_mass * child_cg_x
                        mass_moment_y += child_mass * child_cg_y
                        mass_moment_z += child_mass * child_cg_z
                        
                        # Transform the inertia to account for parallel axis theorem
                        # For simplicity, we'll just keep the basic inertia for now
                        total_ixx += child_results["total_ixx"]
                        total_iyy += child_results["total_iyy"]
                        total_izz += child_results["total_izz"]
                        total_ixy += child_results.get("total_ixy", 0.0)
                        total_ixz += child_results.get("total_ixz", 0.0)
                        total_iyz += child_results.get("total_iyz", 0.0)
                except Exception as e:
                    print(f"Warning: Error getting mass analysis from child {child.name}: {e}")

        # Calculate CG (avoid division by zero)
        cg_x = mass_moment_x / total_mass if total_mass > 0 else 0
        cg_y = mass_moment_y / total_mass if total_mass > 0 else 0
        cg_z = mass_moment_z / total_mass if total_mass > 0 else 0

        # Store and return results
        self.results = {
            "total_mass": total_mass,
            "cg_x": cg_x,
            "cg_y": cg_y,
            "cg_z": cg_z,
            "total_ixx": total_ixx,
            "total_iyy": total_iyy,
            "total_izz": total_izz,
            "total_ixy": total_ixy,
            "total_ixz": total_ixz,
            "total_iyz": total_iyz
        }
        
        return self.results 