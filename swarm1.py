# This Python file uses the following encoding: utf-8
# Copyright 2015 Tin Arm Engineering AB
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""
   This is a sample using the routing library python wrapper to solve a
   A description of the problem can be found here:
   http://en.wikipedia.org/wiki/Vehicle_routing_problem.
   The variant which is tackled by this model includes a fligth time dimension with a penalty cost if 	 tasks are not performed.
   To help explore the problem, two classes are provided tasks() and 
   Drones():
   Distances are computed using the Great Circle distances. Distances are in km
   and times in seconds.
   A function for the displaying of the vehicle plan
   display_vehicle_output
   The optimization engine uses local search to improve solutions, first
   solutions being generated using a cheapest addition heuristic.
   Numpy and Matplotlib are required for the problem creation and display.
"""
import os
import sys
import numpy as np
from matplotlib import pyplot as plt
from collections import namedtuple
from ortools.constraint_solver import pywrapcp
from ortools.constraint_solver import routing_enums_pb2
from datetime import datetime, timedelta

def pointiterator(fra,til,steps):
    '''
    generator, like range() but uses number of steps,
    and handles edge cases (like -180 to +180 crossover)
    '''
    val = fra
    if til < fra:
        til += 360.0
    stepsize = (til - fra)/steps
    while val < til + stepsize:
        if (val > 180.0):
            yield val - 360.0
        else:
            yield val
        val += stepsize
class Tasks():
    """
          A class that generates and holds tasks information.
          Randomly normally distribute a number of tasks and customers within
          a region described by a rectangle.  Generate a random demand for each
          customer. Generate a random time window for each customer.
          May either be initiated with the extents, as a dictionary describing
          two corners of a rectangle in latitude and longitude OR as a center
          point (lat, lon), and box_size in km.  The default arguments are for a
          10 x 10 km square centered in Sheffield).
          Args: extents (Optional[Dict]): A dictionary describing a rectangle in
          latitude and longitude with the keys 'llcrnrlat', 'llcrnrlon' &
          'urcrnrlat' & 'urcrnrlat'  center (Optional(Tuple): A tuple of
          (latitude, longitude) describing the centre of the rectangle.  box_size
          (Optional float: The length in km of the box's sides.  num_stops (int):
          The number of tasks, including the depots that are placed normally
          distributed in the rectangle.  min_demand (int): Lower limit on the
          randomly generated demand at each customer.  max_demand (int): Upper
          limit on the randomly generated demand at each customer.
              min_tw: shortest random time window for a customer, in hours.
              max_tw: longest random time window for a customer, in hours.
          Examples: To place 100 tasks randomly within 100 km x 100 km
          rectangle, centered in the default location, with a random demand of
          between 5 and 10 units:  >>> tasks = tasks(num_stops=100,
          box_size=100, ...                 min_demand=5, max_demand=10)
          alternatively, to place 75 tasks in the same area with default
          arguments for demand:  >>> extents = {'urcrnrlon': 0.03403, 'llcrnrlon':
          -2.98325, ...     'urcrnrlat': 54.28127, 'llcrnrlat': 52.48150} >>>
          tasks = tasks(num_stops=75, extents=extents)
    """


    def __init__(self,
                 extents=None,
                 center=(35.1395626,33.3526203), #KIOSCOE
                 box_size=10,
                 num_stops=0,
                 min_demand=None,
                 max_demand=None):
        self.number = num_stops  #: The number of customers and depots
        #: Location, a named tuple for customers.
        Location = namedtuple('Location', ['lat', 'lon'])
        if extents is not None:
            self.extents = extents  #: The lower left and upper right points
            #: Location[lat,lon]: the centre point of the area.
            self.center = Location(
                extents['urcrnrlat'] -
                0.5 * (extents['urcrnrlat'] - extents['llcrnrlat']),
                extents['urcrnrlon'] -
                0.5 * (extents['urcrnrlon'] - extents['llcrnrlon']))
        else:
            #: Location[lat,lon]: the centre point of the area.
            (clat, clon) = self.center = Location(center[0], center[1])
            rad_earth = 6367  # km
            circ_earth = np.pi * rad_earth
            #: The lower left and upper right points
            self.extents = {
                'llcrnrlon': (
                        clon - 180 * box_size / (circ_earth * np.cos(np.deg2rad(clat)))),
                'llcrnrlat':
                    clat - 180 * box_size / circ_earth,
                'urcrnrlon': (
                        clon + 180 * box_size / (circ_earth * np.cos(np.deg2rad(clat)))),
                'urcrnrlat':
                    clat + 180 * box_size / circ_earth
            }
        # The 'name' of the stop, indexed from 0 to num_stops-1
        stops = np.array(range(0, num_stops))

        # normaly distributed random distribution of stops within the box
        stdv = 3  # the number of standard deviations 99.9% will be within +-3
		

        """
         #evacuation path
                   
        print x
        lats = (
                    self.extents['llcrnrlat'] +x*
                    (self.extents['urcrnrlat'] - self.extents['llcrnrlat']) / stdv)
        lons = (
                self.extents['llcrnrlon'] + +x *
                (self.extents['urcrnrlon'] - self.extents['llcrnrlon']) / stdv)
        """
        x= np.linspace(0,1,num_stops)
        
        #lats = np.linspace(34,35.1395626,num_stops)
        #lons = np.linspace(32,33.3526203,num_stops)
        xiter = pointiterator(float(sys.argv[1]),float(sys.argv[2]),12) #      xiter = pointiterator(35.1395626,35.3395,9)
        yiter = pointiterator(float(sys.argv[3]),float(sys.argv[4]),10) #  yiter = pointiterator(33.3526203,33.552,10)
        # make grid
        xx=np.fromiter(xiter,dtype=np.float)
        yy=np.fromiter(yiter,dtype=np.float)
        xo, yo = np.meshgrid(xx,yy,indexing='xy')

        lats=np.ravel(xo)

        lons=np.ravel(yo)

        # uniformly distributed integer demands.
        demmands = np.random.randint(min_demand, max_demand, num_stops+1)

        print(lats)
	with open('lats','w') as f:
	   f.write("\n".join(str(item) for item in lats)+ "\n")
	with open('lons','w') as f:
	   f.write("\n".join(str(item) for item in lons)+ "\n")
        print(lons)

        self.time_horizon = 5*60
        # A 1 hour period. #time
        print(self.time_horizon)

        # A named tuple for the customer
        Task = namedtuple(
            'Task',
            [
                'index',  # the index of the stop
                'demand',  # the demand for the stop
                'lat',  # the latitude of the stop
                'lon' # the longitude of the stop
            ])

        self.tasks = [
            Task(idx, dem, lat, lon)
            for idx, dem, lat, lon in zip(
                stops, demmands, lats, lons, )
        ]

        # The number of seconds needed to 'unload' 1 unit of goods.
        self.service_time_per_dem = 5  # seconds



    def central_start_node(self, invert=False):
        """
            Return a random starting node, with probability weighted by distance
            from the centre of the extents, so that a central starting node is
            likely.
            Args: invert (Optional bool): When True, a peripheral starting node is
            most likely.
            Returns:
                int: a node index.
            Examples:
                >>> tasks.central_start_node(invert=True)
                42
            """
        num_nodes = len(self.tasks)
        dist = np.empty((num_nodes, 1))
        for idx_to in range(num_nodes):
            dist[idx_to] = self._haversine(self.center.lon, self.center.lat,
                                           self.tasks[idx_to].lon,
                                           self.tasks[idx_to].lat)
        furthest = np.max(dist)

        if invert:
            prob = dist * 1.0 / sum(dist)
        else:
            prob = (furthest - dist * 1.0) / sum(furthest - dist)
        indexes = np.array([range(num_nodes)])
        start_node = np.random.choice(
            indexes.flatten(), size=1, replace=True, p=prob.flatten())
        return 0; #start_node[0]

    def make_distance_mat(self, method='haversine'):
        """
            Return a distance matrix and make it a member of Customer, using the
            method given in the call. Currently only Haversine (GC distance) is
            implemented, but Manhattan, or using a maps API could be added here.
            Raises an AssertionError for all other methods.
            Args: method (Optional[str]): method of distance calculation to use. The
            Haversine formula is the only method implemented.
            Returns:
                Numpy array of node to node distances.
            Examples:
                >>> dist_mat = tasks.make_distance_mat(method='haversine')
                >>> dist_mat = tasks.make_distance_mat(method='manhattan')
                AssertionError
            """
        self.distmat = np.zeros((self.number, self.number))
        methods = {'haversine': self._haversine}
        assert (method in methods)
        for frm_idx in range(self.number):
            for to_idx in range(self.number):
                if frm_idx != to_idx:
                    frm_c = self.tasks[frm_idx]
                    to_c = self.tasks[to_idx]
                    self.distmat[frm_idx, to_idx] = self._haversine(
                        frm_c.lon, frm_c.lat, to_c.lon, to_c.lat)
        return (self.distmat)

    def _haversine(self, lon1, lat1, lon2, lat2):
        """
            Calculate the great circle distance between two points
            on the earth specified in decimal degrees of latitude and longitude.
            https://en.wikipedia.org/wiki/Haversine_formula
            Args:
                lon1: longitude of pt 1,
                lat1: latitude of pt 1,
                lon2: longitude of pt 2,
                lat2: latitude of pt 2
            Returns:
                the distace in km between pt1 and pt2
            """
        # convert decimal degrees to radians
        lon1, lat1, lon2, lat2 = map(np.radians, [lon1, lat1, lon2, lat2])

        # haversine formula
        dlon = lon2 - lon1
        dlat = lat2 - lat1
        a = (
                np.sin(dlat / 2)**2 + np.cos(lat1) * np.cos(lat2) * np.sin(dlon / 2)**2)
        c = 2 * np.arcsin(np.sqrt(a))

        # 6367 km is the radius of the Earth
        km = 6367 * c
        return km

    def get_total_demand(self):
        """
            Return the total demand of all tasks.
            """
        return (sum([c.demand for c in self.tasks]))

    def return_dist_callback(self, **kwargs):
        """
            Return a callback function for the distance matrix.
            Args: **kwargs: Arbitrary keyword arguments passed on to
            make_distance_mat()
            Returns:
                function: dist_return(a,b) A function that takes the 'from' node
                    index and the 'to' node index and returns the distance in km.
            """
        self.make_distance_mat(**kwargs)

        def dist_return(a, b):
            return (self.distmat[a][b])

        return dist_return

    def return_dem_callback(self):
        """
            Return a callback function that gives the demands.
            Returns:
                function: dem_return(a,b) A function that takes the 'from' node
                    index and the 'to' node index and returns the distance in km.
            """

        def dem_return(a, b):
            return (self.tasks[a].demand)

        return dem_return

    def zero_depot_demands(self, depot):
        """
            Zero out the demands and time windows of depot.  The Depots do not have
            demands or time windows so this function clears them.
            Args:  depot (int): index of the stop to modify into a depot.
            Examples:  >>> tasks.zero_depot_demands(5) >>>
            tasks.tasks[5].demand == 0 True
        """
        start_depot = self.tasks[depot]
        self.tasks[depot] = start_depot._replace(
            demand=0)

    def make_service_time_call_callback(self):
        """
            Return a callback function that provides the time spent servicing the
            customer.  Here is it proportional to the demand given by
            self.service_time_per_dem, default 300 seconds per unit demand.
            Returns:
                function [dem_return(a, b)]: A function that takes the from/a node
                    index and the to/b node index and returns the service time at a
            """

        def service_time_return(a, b):
            # return (self.tasks[a].demand * self.service_time_per_dem)
            return 10
        return service_time_return

    def make_transit_time_callback(self, speed_kmph=30):
        """
            Creates a callback function for transit time. Assuming an average
            speed of speed_kmph
            Args:
                speed_kmph: the average speed in km/h
            Returns:
                function [tranit_time_return(a, b)]: A function that takes the
                    from/a node index and the to/b node index and returns the
                    tranit time from a to b.
            """

        def tranit_time_return(a, b):
            return (self.distmat[a][b] / (speed_kmph * 1.0 / 60**2))

        return tranit_time_return
    def make_transit_fuel_callback(self, fuel_cost_km=1.7):
        """
            Creates a callback function for transit fuel. Assuming an average
            speed of speed_kmph
            Args:
                speed_kmph: the average speed in km/h
            Returns:
                function [tranit_fuel_return(a, b)]: A function that takes the
                    from/a node index and the to/b node index and returns the
                    fuel cost from a to b.
            """

        def tranit_fuel_return(a, b):
            return (self.distmat[a][b] / (fuel_cost_km * 1.0 / 60**2))

        return tranit_fuel_return

class Drones():
    """
      A Class to create and hold vehicle information.
      The Vehicles in a CVRPTW problem service the tasks and belong to a
      depot. The class Vehicles creates a list of named tuples describing the
      Vehicles.  The main characteristics are the vehicle capacity, fixed cost,
      and cost per km.  The fixed cost of using a certain type of vehicles can be
      higher or lower than others. If a vehicle is used, i.e. this vehicle serves
      at least one node, then this cost is added to the objective function.
      Note:
          If numpy arrays are given for capacity and cost, then they must be of
          the same length, and the number of vehicles are infered from them.
          If scalars are given, the fleet is homogenious, and the number of
          vehicles is determied by number.
      Args: capacity (scalar or numpy array): The integer capacity of demand
      units.  cost (scalar or numpy array): The fixed cost of the vehicle.  number
      (Optional [int]): The number of vehicles in a homogenious fleet.
    """

    def __init__(self, fuel=100, capacity=100, cost=100, number=None):

        Drone = namedtuple('Drone', ['index', 'fuel','capacity', 'cost'])
        self.fuel=183370

        if number is None:
            self.number = np.size(capacity)
        else:
            self.number = number
        idxs = np.array(range(0, self.number))

        if np.isscalar(capacity):
            capacities = capacity * np.ones_like(idxs)
        elif np.size(capacity) != np.size(capacity):
            print('capacity is neither scalar, nor the same size as num!')
        else:
            capacities = capacity
        fuels= fuel
        if np.isscalar(cost):
            costs = cost * np.ones_like(idxs)
        elif np.size(cost) != self.number:
            print(np.size(cost))
            print('cost is neither scalar, nor the same size as num!')
        else:
            costs = cost

        self.drones = [
            Drone(idx, fuel, capacity, cost)
            for idx, fuel, capacity, cost in zip(idxs, fuels, capacities, costs)
        ]

    def get_total_fuel(self):
        return (sum([c.fuel for c in self.drones]))


    def get_total_capacity(self):
        return (sum([c.capacity for c in self.drones]))

    def return_starting_callback(self, tasks, sameStartFinish=False): #maria = same depot for every vehicle
        # create a different starting and finishing depot for each vehicle
        self.starts = [
            int(tasks.central_start_node()) for o in range(self.number) #
        ]
        if sameStartFinish:
            self.ends = self.starts
        else:
            self.ends = [
                int(tasks.central_start_node(invert=True))
                for o in range(self.number)
            ]
        # the depots will not have demands, so zero them.
        for depot in self.starts:
            tasks.zero_depot_demands(depot)
        for depot in self.ends:
            tasks.zero_depot_demands(depot)

        def start_return(v):
            return (self.starts[v])

        return start_return


def discrete_cmap(N, base_cmap=None):
    """
      Create an N-bin discrete colormap from the specified input map
      """
    # Note that if base_cmap is a string or None, you can simply do
    #    return plt.cm.get_cmap(base_cmap, N)
    # The following works for string, None, or a colormap instance:

    base = plt.cm.get_cmap(base_cmap)
    color_list = base(np.linspace(0, 1, N))
    cmap_name = base.name + str(N)
    return base.from_list(cmap_name, color_list, N)


def vehicle_output_string(routing, plan):
    """
      Return a string displaying the output of the routing instance and
      assignment (plan).
      Args: routing (ortools.constraint_solver.pywrapcp.RoutingModel): routing.
      plan (ortools.constraint_solver.pywrapcp.Assignment): the assignment.
      Returns:
          (string) plan_output: describing each vehicle's plan.
          (List) dropped: list of dropped orders.
      """
    dropped = []
    for order in range(routing.Size()):
        if (plan.Value(routing.NextVar(order)) == order):
            dropped.append(str(order))

    #capacity_dimension = routing.GetDimensionOrDie('Capacity')
    #fuel_dimension = routing.GetDimensionOrDie('Fuel')
    time_dimension = routing.GetDimensionOrDie('Time')


    plan_output = ''
    timef=0;
    for route_number in range(routing.vehicles()):
        order = routing.Start(route_number)
        plan_output += 'Route for V {0}:'.format(route_number)
        if routing.IsEnd(plan.Value(routing.NextVar(order))):
            plan_output += ' Empty \n'
        else:
            while True:
                # load_var = capacity_dimension.CumulVar(order)
                time_var = time_dimension.CumulVar(order)
                #fuel_var= fuel_dimension.CumulVar(order)
                #   plan_output += \
                #      ' {order} Load({load}) Time({tmin}, {tmax}) -> '.format(
                #          order=order,
                #          load=plan.Value(load_var),
                #          tmin=str(timedelta(seconds=plan.Min(time_var))),
                #          tmax=str(timedelta(seconds=plan.Max(time_var))))
                plan_output += \
                    ' {order} @ {tmin}'.format(
                        order=order,
                        # fuel=plan.Value(fuel_var),
                        tmin=str(timedelta(seconds=plan.Min(time_var)))
                    )
                with open("route"+str(route_number),'a') as f:
		    f.write(str(order)+ '\n')
	        if routing.IsEnd(order):
                    timef+=plan.Min(time_var)
                    plan_output += ' EndRoute {0} ,  {1}. \n'.format(route_number,str(timedelta(seconds=timef)))
                    break
                order = plan.Value(routing.NextVar(order))
        plan_output += '\n'
    plan_output += ' Total Flying Time {0}. \n'.format(str(timedelta(seconds=timef)))

    return (plan_output, dropped)


def build_vehicle_route(routing, plan, tasks, veh_number):
    """
      Build a route for a vehicle by starting at the strat node and
      continuing to the end node.
      Args: routing (ortools.constraint_solver.pywrapcp.RoutingModel): routing.
      plan (ortools.constraint_solver.pywrapcp.Assignment): the assignment.
      tasks (tasks): the tasks instance.  veh_number (int): index of
      the vehicle
      Returns:
          (List) route: indexes of the tasks for vehicle veh_number
      """
    veh_used = routing.IsVehicleUsed(plan, veh_number)


    print('Vehicle {0} is used {1}'.format(veh_number, veh_used))
    if veh_used:
        route = []
        node = routing.Start(veh_number)  # Get the starting node index
        route.append(tasks.tasks[routing.IndexToNode(node)])
        while not routing.IsEnd(node):
            route.append(tasks.tasks[routing.IndexToNode(node)])
            node = plan.Value(routing.NextVar(node))

        route.append(tasks.tasks[routing.IndexToNode(node)])
        return route
    else:
        return None


def plot_vehicle_routes(veh_route, ax1, tasks, vehicles):
    """
      Plot the vehicle routes on matplotlib axis ax1.
      Args: veh_route (dict): a dictionary of routes keyed by vehicle idx.  ax1
      (matplotlib.axes._subplots.AxesSubplot): Matplotlib axes  tasks
      (tasks): the tasks instance.  vehicles (Vehicles): the vehicles
      instance.
    """
    veh_used = [v for v in veh_route if veh_route[v] is not None]

    cmap = discrete_cmap(vehicles.number + 2, 'nipy_spectral')

    for c in tasks.tasks:
        """ax1.annotate(
            'd{dem}@{num} '.format(
                dem=c.demand, num=c.index),
            xy=(c.lon,c.lat),
            xytext=(10, 10),
            xycoords='data',
            textcoords='offset points',
            arrowprops=dict(
                arrowstyle='->',
                connectionstyle='angle3,angleA=90,angleB=0',
                shrinkA=0.02),
        )
        """
    for veh_number in veh_used:

        lats, lons = zip(*[(c.lat, c.lon) for c in veh_route[veh_number]])
        lats = np.array(lats)
        lons = np.array(lons)
        s_dep = tasks.tasks[vehicles.starts[veh_number]]
        s_fin = tasks.tasks[vehicles.ends[veh_number]]


        ax1.annotate(
            'depot',
            xy=(s_dep.lon, s_dep.lat),
            xytext=(10, 10),
            textcoords='offset points',
            arrowprops=dict(
                arrowstyle='->',
                connectionstyle='angle3,angleA=90,angleB=0',
                shrinkA=0.02),
        )
	

        ax1.quiver(
            lons[:-1],
            lats[:-1],
            lons[1:] - lons[:-1],
            lats[1:] - lats[:-1],
            scale_units='xy',
            angles='xy',
            scale=1,
            color=cmap(veh_number + 1))


def main():
    # Create a set of customer, (and depot) stops.
    tasks = Tasks(
        num_stops=90,
        min_demand=0,
        max_demand=1,
        box_size=0.2)

    # Create callback fns for distances, demands, service and transit-times.
    dist_fn = tasks.return_dist_callback()
    dem_fn = tasks.return_dem_callback()
    fuel_fn = tasks.make_transit_fuel_callback()
    serv_time_fn = tasks.make_service_time_call_callback()
    transit_time_fn = tasks.make_transit_time_callback()

    def tot_time_fn(a, b):
        """
            The time function we want is both transit time and service time.
            """
        return serv_time_fn(a, b) + transit_time_fn(a, b)

    # Create a list of inhomgenious vehicle capacities as integer units.
    capacity = [24,24,24,24]
    fuel = [24,24,24,24]

    # Create a list of inhomogenious fixed vehicle costs.
    cost = [int(100 + 2 * np.sqrt(c)) for c in capacity]

    # Create a set of vehicles, the number set by the length of capacity.
    drones = Drones(capacity=capacity, cost=cost , fuel=fuel)

    # check to see that the problem is feasible, if we don't have enough
    # vehicles to cover the demand, there is no point in going further.
    assert (tasks.get_total_demand() < drones.get_total_fuel())


    # Set the starting nodes, and create a callback fn for the starting node.
    start_fn = drones.return_starting_callback(tasks, sameStartFinish=True)

    # Set model parameters
    model_parameters = pywrapcp.RoutingModel.DefaultModelParameters()

    # The solver parameters can be accessed from the model parameters. For example :
    #   model_parameters.solver_parameters.CopyFrom(
    #       pywrapcp.Solver.DefaultSolverParameters())
    #       pywrapcp.Solver.DefaultSolverParameters())
    #    model_parameters.solver_parameters.trace_propagation = True

    # Make the routing model instance.
    routing = pywrapcp.RoutingModel(
        tasks.number,  # int number
        drones.number,  # int number
        drones.starts,  # List of int start depot
        drones.ends,  # List of int end depot
        model_parameters)

    parameters = routing.DefaultSearchParameters()
    # Setting first solution heuristic (cheapest addition).
    parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
    # Disabling Large Neighborhood Search, (this is the default behaviour)
    parameters.local_search_operators.use_path_lns = False
    parameters.local_search_operators.use_inactive_lns = False
    # Routing: forbids use of TSPOpt neighborhood,
    parameters.local_search_operators.use_tsp_opt = False

    parameters.time_limit_ms = 1 * 1000  # 10 seconds
    parameters.solution_limit=2000;
    parameters.use_light_propagation = False
    # parameters.log_search = True

    # Set the cost function (distance callback) for each arc, homogenious for
    # all vehicles.
    routing.SetArcCostEvaluatorOfAllVehicles(dist_fn)

    # Add a dimension for time and a limit on the total time_horizon
    #set maximum flight time  per each drone
    routing.AddDimension(
        tot_time_fn,  # total time function callback
        0,
        tasks.time_horizon,
        True,
        'Time')
    null_fuel_slack = 0
    #    routing.AddDimension(
    #    fuel_fn, #fuel callback,
    #    0,
    #    drones.fuel,
    #   True,
    #    'Fuel')
    time_dimension_dimension = routing.GetDimensionOrDie('Time')

    time_dimension_dimension.SetSpanCostCoefficientForAllVehicles(100)

    """
       To allow the dropping of orders, we add disjunctions to all the customer
      nodes. Each disjunction is a list of 1 index, which allows that customer to
      be active or not, with a penalty if not. The penalty should be larger
      than the cost of servicing that customer, or it will always be dropped!
      """
    # To add disjunctions just to the tasks, make a list of non-depots.
    non_depot = set(range(tasks.number))
    non_depot.difference_update(drones.starts)
    non_depot.difference_update(drones.ends)
    penalty = 80000  # The cost for dropping a node from the plan.
    nodes = [routing.AddDisjunction([int(c)], penalty) for c in non_depot]

    # This is how you would implement partial routes if you already knew part
    # of a feasible solution for example:
    # partial = np.random.choice(list(non_depot), size=(4,5), replace=False)

    # routing.CloseModel()
    # partial_list = [partial[0,:].tolist(),
    #                 partial[1,:].tolist(),
    #                 partial[2,:].tolist(),
    #                 partial[3,:].tolist(),
    #                 [],[],[],[]]
    # print(routing.ApplyLocksToAllVehicles(partial_list, False))
    fig = plt.figure("Swarm Routes - Monitoring with and without constrain")
    # Solve the problem !
    assignment = routing.SolveWithParameters(parameters)
    # Plotting of the routes in matplotlib.


    # The rest is all optional for saving, printing or plotting the solution.
    if assignment:
        # save the assignment, (Google Protobuf format)
        save_file_base = os.path.realpath(__file__).split('.')[0]
        if routing.WriteAssignment(save_file_base + '_assignment.ass'):
            print('succesfully wrote assignment to file ' + save_file_base +
                  '_assignment.ass')

        print('The Objective Value is {0}'.format(assignment.ObjectiveValue()))

        plan_output, dropped = vehicle_output_string(routing, assignment)
        print(plan_output)
        print('dropped nodes: ' + ', '.join(dropped))
	with open('plan_output','a') as f:
	   f.write(plan_output)
	   f.write('dropped nodes: ' + ', '.join(dropped))
        # you could print debug information like this:
        # print(routing.DebugOutputAssignment(assignment, 'Capacity'))

        vehicle_routes = {}
        for veh in range(drones.number):
            vehicle_routes[veh] = build_vehicle_route(routing, assignment, tasks,
                                                      veh)

        ax =fig.add_subplot(2, 1, 1)
        # Plot all the nodes as black dots.
        clon, clat = zip(*[(c.lon, c.lat) for c in tasks.tasks])
        ax.plot(clon, clat, 'k.')

        # plot the routes as arrows
        #  print vehicle_routes
        plot_vehicle_routes(vehicle_routes, ax, tasks, drones)
        plt.ylabel('without constrain')
	plt.show()
    else:
        print('No assignment')

if __name__ == '__main__':
    main()
