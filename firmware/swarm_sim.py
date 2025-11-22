# swarm_sim.py - Antares Swarm Dynamics Simulator (v0.1)
# Basic Python model for magnetic latching and autonomy
# Run with: python swarm_sim.py

import numpy as np
import matplotlib.pyplot as plt

class Drone:
    def __init__(self, id: int, pos: np.ndarray, lift_capacity: float = 2.4):
        self.id = id
        self.pos = pos
        self.lift_capacity = lift_capacity
        self.connected_to = []

    def distance_to(self, other: 'Drone') -> float:
        return np.linalg.norm(self.pos - other.pos)

class AntaresSwarm:
    def __init__(self, num_drones: int = 500):
        self.drones = []
        for i in range(num_drones):
            pos = np.random.uniform(-5, 5, 3)
            self.drones.append(Drone(i, pos))

    def simulate_latching(self, max_distance: float = 0.5) -> int:
        connections = 0
        for i, drone in enumerate(self.drones):
            for j in range(i + 1, len(self.drones)):
                if drone.distance_to(self.drones[j]) < max_distance:
                    drone.connected_to.append(self.drones[j].id)
                    self.drones[j].connected_to.append(drone.id)
                    connections += 1
        return connections

    def total_lift(self) -> float:
        visited = set()
        total = 0
        for drone in self.drones:
            if drone.id not in visited:
                cluster_size = 1 + len([c for c in drone.connected_to if c not in visited])
                total += cluster_size * drone.lift_capacity
                visited.update([drone.id] + drone.connected_to)
        return total

    def plot_swarm(self):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        for drone in self.drones:
            ax.scatter(drone.pos[0], drone.pos[1], drone.pos[2], c='red', s=10)
        ax.set_title('Antares Swarm - Initial Positions')
        plt.show()

if __name__ == "__main__":
    swarm = AntaresSwarm(500)
    swarm.plot_swarm()
    connections = swarm.simulate_latching()
    lift = swarm.total_lift()
    print(f"Simulated {connections} connections, {lift:.1f} kg total lift")
