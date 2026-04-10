from environment import Environment
from drone import Drone
from task import Task
from map_drawer import OptimizedMapViewer

if __name__ == "__main__":
    env = Environment('data/map/part_of_yangpu.osm')

    env.drones = [Drone(357600.574872369, 3462308.772003661)]

    env.assign_task(env.drones[0], Task(1, 1, (357400.574872369, 3462308.772003661)))

    viewer = OptimizedMapViewer('data/map/part_of_yangpu.osm')
    for i in range(100):
        viewer.render(env.drones)
    while True:
        env.update()
        viewer.render(env.drones)