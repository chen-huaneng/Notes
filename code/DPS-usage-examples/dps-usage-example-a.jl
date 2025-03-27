using TSPDrone
n = 10 
x = rand(n); y = rand(n);
truck_cost_factor = 1.0 
drone_cost_factor = 0.5
result = solve_tspd(x, y, truck_cost_factor, drone_cost_factor)
@show result.total_cost;
@show result.truck_route;
@show result.drone_route;