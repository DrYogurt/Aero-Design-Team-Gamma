from global_variables.solver import EquationSystem
from global_variables.registry import VariableRegistry

registry = VariableRegistry("aero_vars.yaml")

system = EquationSystem(registry, targets={'R'})
#print(system.equations)
new_definitions = system.complete_definitions(variables="R")
print(new_definitions)


