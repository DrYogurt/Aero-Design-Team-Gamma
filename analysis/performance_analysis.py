from global_variables.solver import EquationSystem
from global_variables.registry import VariableRegistry, Variable 

registry = VariableRegistry("aero_vars.yaml")


 # Initialize registry
registry = VariableRegistry("aero_vars.yaml")

    # Create solvers
takeoff_solver = EquationSystem(registry, {"S_TO"})
range_solver = EquationSystem(registry, {"R"})
ceiling_solver = EquationSystem(registry, {"sigma_max"})

print("Required inputs for takeoff:")
print(takeoff_solver.inputs)
print("\nRequired inputs for range:")
print(range_solver.inputs)
print("\nRequired inputs for ceiling:")
print(ceiling_solver.inputs)

    # Create solvers
takeoff_func = takeoff_solver.create_solver()["S_TO"]
range_func = range_solver.create_solver()["R"]
ceiling_func = ceiling_solver.create_solver()["sigma_max"]

print(f"Takeoff Function:\n{takeoff_func}")
print(f"Range Function:\n{range_func}")
print(f"Ceiling Function:\n{ceiling_func}")
