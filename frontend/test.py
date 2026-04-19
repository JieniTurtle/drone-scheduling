from environment import Environment
from scheduler import GreedyScheduler

if __name__ == "__main__":
    env = Environment('data/map/part_of_yangpu.osm', visualize=True)


    frame_count = 0
    running = True
    observations = env.reset()  # 获取初始状态
    while running:
        action = GreedyScheduler.schedule_for_drone(observations)  # 获取调度决策
        observations, reward, running, _ = env.step(action)

        print(f"Frame: {frame_count}, Reward: {reward:.2f}")
        
        
        # 每60帧（约1秒）输出一次状态
        if frame_count % 60 == 0:
            free_drones = sum(1 for d in env.drones if d.is_free)
            busy_drones = sum(1 for d in env.drones if not d.is_free)
            # print(f"[Status] Unassigned tasks: {len(unassigned_tasks)}, Free drones: {free_drones}, Busy drones: {busy_drones}")
        