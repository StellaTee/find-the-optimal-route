# find-the-optimal-route

This project implements the Dijkstra algorithm to find the most optimal route under the following constraints. The constraints include whether to pick up a friend, the time required to travel to the destination, and the shortest total driving time. If giving a ride to a friend increases the total driving time, I will not give a ride. My solution is a multi-layer graph approach.

Please note that there is a law in the country that prohibits driving in a carpool lane while alone in the car.

My assumption of this world:
- all the roads are connected
- no passengers will be at the start and the target place
- there is no road that will only have a carpool lane
- It is always a one-way road (directed graph)

