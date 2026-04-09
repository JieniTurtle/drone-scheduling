class Drone:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.tasks = []
        self.scheduled_position = []
        self.is_free = True

    def schedule_route(self, position):
        self.scheduled_position = position
        self.is_free = False

    def update(self, time_step=0.1):
        v = 10
        max_distance = v * time_step
        if self.scheduled_position:
            # Get the next target position
            target_x, target_y = self.scheduled_position[0]
            
            # Calculate direction vector
            dx = target_x - self.x
            dy = target_y - self.y
            
            # Calculate distance to target
            distance = (dx**2 + dy**2)**0.5
            
            if distance <= max_distance:
                # If we're close enough to target, move directly to it
                self.x = target_x
                self.y = target_y
                # Remove this target from schedule as we've reached it
                self.scheduled_position.pop(0)
                if not self.scheduled_position:
                    self.is_free = True
            else:
                # Move a step towards the target
                # Normalize the direction and multiply by time_step
                self.x += (dx / distance) * max_distance
                self.y += (dy / distance) * max_distance

    def get_position(self):
        return (self.x, self.y)