from typing import Dict, List, Set, Tuple
from sympy import solve, symbols, lambdify
import networkx as nx

class EquationGraph:
    """Builds a graph of equations and variables to determine solution order."""
    
    def __init__(self, registry):
        self.registry = registry
        self.graph = nx.DiGraph()
        self._build_graph()
        
    def _build_graph(self):
        """Build directed graph of variable dependencies."""
        for symbol, (path, var) in self.registry._symbol_map.items():
            self.graph.add_node(symbol, variable=var)
            if var.definitions:
                for eq in var.definitions:
                    for dep_symbol in self.registry._extract_symbols(eq):
                        if dep_symbol != symbol:  # Avoid self-loops
                            self.graph.add_edge(dep_symbol, symbol)
    
    def get_required_variables(self, targets: Set[str]) -> Tuple[Set[str], Set[str]]:
        """
        Given target variables, return sets of (required_inputs, intermediate_vars).
        """
        required = set()
        intermediates = set()
        
        for target in targets:
            # Get all predecessors for this target
            predecessors = nx.ancestors(self.graph, target)
            required.update(predecessors)
            # Get variables that are both required and have definitions
            intermediates.update(
                sym for sym in predecessors 
                if self.registry._symbol_map[sym][1].definitions
            )
        
        # Remove intermediates from required inputs
        required -= intermediates
        return required, intermediates

class EquationSystem:
    """Manages a system of equations from the registry."""
    
    def __init__(self, registry, targets: Set[str]):
        self.registry = registry
        self.targets = targets
        self.graph = EquationGraph(registry)
        self.inputs, self.intermediates = self.graph.get_required_variables(targets)
        self.equations = self._collect_equations()
        
    def _collect_equations(self) -> Dict[str, List[str]]:
        """Collect all equations needed to solve for targets."""
        equations = {}
        for var in self.intermediates | self.targets:
            var_obj = self.registry._symbol_map[var][1]
            if var_obj.definitions:
                equations[var] = var_obj.definitions
        return equations
    
    def create_solver(self) -> Dict[str, callable]:
        """Create lambda functions for each target variable."""
        # Create sympy symbols for all variables
        syms = {
            name: symbols(name.replace('{', '').replace('}', '')) 
            for name in self.inputs | self.intermediates | self.targets
        }
        
        # Create system of equations
        eqns = []
        for var, def_list in self.equations.items():
            for def_eq in def_list:
                # Convert string equation to sympy expression
                lhs = syms[var]
                # Check for curly braces in variable names
                if any('{' in sym_name or '}' in sym_name for sym_name in syms):
                    raise ValueError("Variable names cannot contain curly braces. Use alternative notation (e.g., 'C_L0' instead of 'C_{L_0}')")
                
                try:
                    rhs = eval(def_eq, {}, syms)
                    eqns.append(lhs - rhs)
                except Exception as e:
                    raise ValueError(f"Failed to parse equation '{def_eq}': {str(e)}")
        
        # Solve system for targets
        solution = solve(eqns, [syms[t] for t in self.targets])
        
        # Create lambda functions
        solvers = {}
        input_syms = [syms[v] for v in sorted(self.inputs)]
        
        for target, expr in zip(self.targets, solution):
            solvers[target] = lambdify(input_syms, expr)
            
        return solvers

# Example usage:
if __name__ == "__main__":
    from registry import VariableRegistry, Variable
    
    # Create registry with some variables
    registry = VariableRegistry("aero_vars.yaml")
    
    registry.aero.coefficients.lift.lift = Variable(
        name="lift coefficient",
        symbol="C_L",
        definitions=["2*W / rho / V**2 / S"]
    )
    
    registry.aero.coefficients.lift.max = Variable(
        name="max lift",
        symbol="C_L_max",
        definitions=["2.5 * C_L_0"]
    )
    
    # Create equation system
    system = EquationSystem(registry, {"C_L_max"})
    
    # Get solvers
    solvers = system.create_solver()
    
    # Show required inputs
    print(f"Required inputs: {system.inputs}")
    
    # Use solver
    if not system.inputs:  # All inputs are defined
        result = solvers["C_L_max"]()
        print(f"C_L_max = {result}")
