import pygame
import math
import random
from pydub import AudioSegment
import simpleaudio as sa

# Initialize Pygame
pygame.init()

# Screen dimensions
WIDTH, HEIGHT = 800, 800
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Balls within Circle Simulation")

# Colors
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)

# Circle boundary
CIRCLE_RADIUS = 350
CIRCLE_CENTER = (WIDTH // 2, HEIGHT // 2)

# Load the full sound file using pydub
full_sound = AudioSegment.from_file("airplane-sound.wav")
segment_duration = 250  # Duration of each segment in milliseconds
num_segments = len(full_sound) // segment_duration
current_segment = 0

# Load the explosion image with transparency
explosion_img = pygame.image.load("explosion.png").convert_alpha()
explosion_img = pygame.transform.scale(explosion_img, (100, 100))

# Variables for displaying the explosion
explosion_displayed = False
explosion_timer = 0
explosion_duration = 10  # Display for 10 frames (adjust as needed)
explosion_position = (0, 0)


# Ball class
class Ball:
    def __init__(self, x, y, radius, dx, dy):
        self.x = x
        self.y = y
        self.radius = radius
        self.dx = dx
        self.dy = dy
        self.color = 255  # Start as white
        self.collision_count = 0  # Track number of collisions

    def move(self):
        self.x += self.dx
        self.y += self.dy

        # Check for collision with the circular boundary
        distance_from_center = math.hypot(self.x - CIRCLE_CENTER[0], self.y - CIRCLE_CENTER[1])
        if distance_from_center + self.radius >= CIRCLE_RADIUS:
            angle = math.atan2(self.y - CIRCLE_CENTER[1], self.x - CIRCLE_CENTER[0])
            overlap = distance_from_center + self.radius - CIRCLE_RADIUS
            self.x -= overlap * math.cos(angle)
            self.y -= overlap * math.sin(angle)
            self.dx = -self.dx
            self.dy = -self.dy

    def draw(self, screen):
        color_value = max(0, self.color)
        pygame.draw.circle(screen, (color_value, color_value, color_value), (int(self.x), int(self.y)), self.radius)

    def collide(self, other):
        distance = math.hypot(self.x - other.x, self.y - other.y)
        return distance < self.radius + other.radius

    def resolve_collision(self, other):
        global current_segment, explosion_displayed, explosion_timer, explosion_position
        normal = (other.x - self.x, other.y - self.y)
        distance = math.hypot(normal[0], normal[1])
        normal = (normal[0] / distance, normal[1] / distance)

        relative_velocity = (self.dx - other.dx, self.dy - other.dy)
        velocity_along_normal = relative_velocity[0] * normal[0] + relative_velocity[1] * normal[1]

        if velocity_along_normal > 0:
            return

        restitution = 0.8  # Less than 1 for inelastic collisions
        j = -(1 + restitution) * velocity_along_normal
        j /= 1 / self.radius + 1 / other.radius

        impulse = (j * normal[0], j * normal[1])

        self.dx -= impulse[0] / self.radius
        self.dy -= impulse[1] / self.radius
        other.dx += impulse[0] / other.radius
        other.dy += impulse[1] / other.radius

        # Apply a damping factor to all velocities
        damping_factor = 0.9
        self.dx *= damping_factor
        self.dy *= damping_factor
        other.dx *= damping_factor
        other.dy *= damping_factor

        # Clip maximum velocity to prevent runaway acceleration
        max_velocity = 10
        self.dx = max(min(self.dx, max_velocity), -max_velocity)
        self.dy = max(min(self.dy, max_velocity), -max_velocity)
        other.dx = max(min(other.dx, max_velocity), -max_velocity)
        other.dy = max(min(other.dy, max_velocity), -max_velocity)

        # Increment collision counts
        self.collision_count += 1
        other.collision_count += 1

        # Play the current collision sound segment
        play_sound_segment()

        # Trigger explosion effect at collision point
        collision_x = (self.x + other.x) / 2
        collision_y = (self.y + other.y) / 2
        explosion_position = (collision_x - explosion_img.get_width() // 2, collision_y - explosion_img.get_height() // 2)
        explosion_displayed = True
        explosion_timer = explosion_duration

    def darken(self):
        self.color -= 5
        self.color = max(self.color, 0)


def play_sound_segment():
    global current_segment
    start_ms = current_segment * segment_duration
    end_ms = start_ms + segment_duration
    segment = full_sound[start_ms:end_ms]
    play_obj = sa.play_buffer(
        segment.raw_data,
        num_channels=segment.channels,
        bytes_per_sample=segment.sample_width,
        sample_rate=segment.frame_rate
    )
    current_segment = (current_segment + 1) % num_segments


def create_new_ball():
    x = random.randint(CIRCLE_CENTER[0] - CIRCLE_RADIUS + 20, CIRCLE_CENTER[0] + CIRCLE_RADIUS - 20)
    y = random.randint(CIRCLE_CENTER[1] - CIRCLE_RADIUS + 20, CIRCLE_CENTER[1] + CIRCLE_RADIUS - 20)
    dx = random.choice([-3, -2, -1, 1, 2, 3])
    dy = random.choice([-3, -2, -1, 1, 2, 3])
    return Ball(x, y, 20, dx, dy)


# Create initial balls with specific positions and velocities
balls = [
    Ball(300, 400, 20, 4, 3),
    Ball(400, 500, 20, -4, -3),
    Ball(500, 600, 20, 3, -4)
]

# Main loop
running = True
clock = pygame.time.Clock()

while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    screen.fill(BLACK)

    # Draw the boundary circle
    pygame.draw.circle(screen, WHITE, CIRCLE_CENTER, CIRCLE_RADIUS, 2)

    # Draw and move the balls
    for ball in balls:
        ball.move()
        ball.draw(screen)

    # Handle collisions and create new balls if needed
    new_balls = []
    balls_to_remove = []
    for i in range(len(balls)):
        for j in range(i + 1, len(balls)):
            if balls[i].collide(balls[j]):
                balls[i].resolve_collision(balls[j])
                balls[i].darken()
                balls[j].darken()
                if len(balls) + len(new_balls) < 20:  # Only add new balls if under the limit
                    new_balls.append(create_new_ball())
                if balls[i].collision_count >= 3:
                    balls_to_remove.append(balls[i])
                if balls[j].collision_count >= 3:
                    balls_to_remove.append(balls[j])

    # Add new balls to the list
    balls.extend(new_balls)

    # Remove balls that have exceeded collision limit
    for ball in balls_to_remove:
        if ball in balls:
            balls.remove(ball)

    # Display the explosion if triggered
    if explosion_displayed:
        screen.blit(explosion_img, explosion_position)
        explosion_timer -= 1
        if explosion_timer <= 0:
            explosion_displayed = False

    pygame.display.flip()
    clock.tick(30)  # Adjust the FPS as needed

pygame.quit()
