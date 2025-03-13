import numpy as np
import pygame

class BrownianMotion:
    def __init__(self, arena_size=500):
        self.arena_size = arena_size
        self.position = np.array([arena_size/2, arena_size/2])
        self.direction = np.random.randint(2*np.pi)
        self.path = [self.position.copy()]

    def move(self):
        dx = np.cos(self.direction)*2
        dy = np.sin(self.direction)*2
        new_position = self.position + np.array([dx, dy])
        
        if not (new_position[0]>=0 and new_position[0]<=self.arena_size):
            self.direction = np.random.randint(2*np.pi)
        elif not (new_position[1]>=0 and new_position[1]<=self.arena_size):
            self.direction = np.random.randint(2*np.pi)
        else:
            self.position = new_position
            self.path.append(self.position.copy())

def simulate():
    pygame.init()
    arena_size = 500
    
    screen = pygame.display.set_mode((arena_size, arena_size))
    pygame.display.set_caption("Brownian Motion Simulation")
    clock = pygame.time.Clock()
    
    robot = BrownianMotion(arena_size)
    running = True
    while running:
        screen.fill((0,0,0))
        
        for i in range(1, len(robot.path)):
            pygame.draw.line(screen, (0,0,255), robot.path[i-1], robot.path[i], 2)
        
        pygame.draw.circle(screen, (0,255,0), robot.position.astype(int), 5)
        pygame.display.flip()
        robot.move()
        
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
                
        clock.tick(60)
    
    pygame.quit()

if __name__ == "__main__":
    simulate()
