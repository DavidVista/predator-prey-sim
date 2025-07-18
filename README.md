# Turtle Spawner

This ROS2 package provides a system that automatically deletes the default turtle and spawns a custom turtle at a specified location.

## Features

- Automatically kills the default turtle (turtle1)
- Spawns a prey turtle named "prey_turtle" at position (5.0, 5.0)
- Makes the prey turtle move randomly while staying within borders
- Spawns a predator turtle named "predator_turtle" at position (1.0, 1.0) after 5 seconds
- Ready for prey-predator simulation development

## Building

From the ROS2 workspace root directory:

```bash
colcon build --packages-select turtle_spawner
source install/setup.bash
```

## Running

### Option 1: Standard Stalking Behavior
```bash
ros2 launch turtle_spawner turtle_spawner.launch.py
```

### Option 2: Energy-Based Hunting (Cheetah Mode)
```bash
ros2 launch turtle_spawner energy_turtle_spawner.launch.py
```

### Option 3: Boids Prey with Standard Predator
```bash
ros2 launch turtle_spawner boids_turtle_spawner.launch.py
```

### Option 4: Boids Prey with Energy Predator (Most Realistic)
```bash
ros2 launch turtle_spawner boids_energy_spawner.launch.py
```

### Option 5: Manual execution
```bash
# Terminal 1: Start turtlesim
ros2 run turtlesim turtlesim_node

# Terminal 2: Start the turtle manager
ros2 run turtle_spawner turtle_manager

# Terminal 3: Start the turtle controller
ros2 run turtle_spawner turtle_controller

# Terminal 4: Start the prey controller (choose one):
ros2 run turtle_spawner turtle_controller            # Random movement
ros2 run turtle_spawner boids_prey_controller        # Boids flocking

# Terminal 5: Start the predator controller (choose one):
ros2 run turtle_spawner predator_controller          # Standard stalking
ros2 run turtle_spawner energy_predator_controller   # Energy-based (cheetah)
```

## What happens

1. The system starts turtlesim (which creates the default turtle1)
2. After 1 second, the turtle manager kills turtle1
3. After 2 more seconds, it spawns "prey_turtle" at position (5.0, 5.0)
4. The turtle controller makes the prey turtle move randomly while staying within borders
5. After 5 more seconds, it spawns "predator_turtle" at position (1.0, 1.0)
6. The predator controller implements stalking behavior to hunt the prey
7. When predator catches prey (within 0.5 units), prey is killed
8. After 3 seconds, prey respawns at random location and hunting continues

## Movement Behavior

- The turtle moves with random linear velocity (0.5 to 2.0 m/s)
- Random angular velocity changes direction (-1.0 to 1.0 rad/s)
- Direction changes randomly every 3 seconds (30% chance)
- When near borders (within 0.5 units), the turtle slows down and turns away
- Border limits: x and y between 0.5 and 10.5

## Current Implementation: Stalking Behavior

The predator implements realistic stalking behavior with three hunting modes:

### Hunting Parameters:
- **Detection Range**: 8.0 units (maximum distance to detect prey)
- **Stalking Range**: 4.0 units (distance where stalking behavior starts)
- **Attack Range**: 1.0 units (distance where predator is very close)
- **Catch Range**: 0.5 units (distance where predator catches prey)

### Speed Modes:
- **Stalking Speed**: 0.5 m/s (slow, stealthy movement when far from prey)
- **Hunting Speed**: 1.5 m/s (medium speed when within stalking range)
- **Attack Speed**: 2.5 m/s (fast speed when very close to prey)

### Behavior:
- **Patrol Mode**: When prey is not detected, predator moves toward center of map
- **Stalking Mode**: When prey is detected but far away, predator moves slowly
- **Hunting Mode**: When prey is within stalking range, predator speeds up
- **Attack Mode**: When prey is very close, predator moves at maximum speed
- **Catch Mode**: When prey is within 0.5 units, predator catches and kills prey
- **Respawn System**: After 3 seconds, prey respawns at random location
- **Border Avoidance**: Predator avoids leaving the turtlesim window

## Boids Prey Behavior (Intelligent Flocking)

The Boids prey controller implements the classic flocking algorithm with predator avoidance:

### Boids Rules:
- **Separation**: Avoid getting too close to borders and other objects
- **Alignment**: Maintain consistent movement direction
- **Cohesion**: Move toward center of safe areas
- **Predator Avoidance**: Flee from predator when detected

### Flocking Parameters:
- **Neighbor Radius**: 3.0 units (for future multi-prey systems)
- **Separation Radius**: 1.5 units (minimum distance)
- **Predator Detection**: 4.0 units (distance to detect and flee)
- **Max Speed**: 2.0 m/s (fleeing speed)
- **Min Speed**: 0.3 m/s (normal movement)

### Behavior Features:
- **Intelligent Movement**: Combines multiple forces for natural behavior
- **Predator Avoidance**: Stronger avoidance when predator is closer
- **Border Avoidance**: Smart navigation away from edges
- **Smooth Movement**: Gradual speed and direction changes
- **Noise Addition**: Small random variations for realism

### Force Weights:
- **Separation**: 1.5 (avoid collisions)
- **Alignment**: 1.0 (maintain direction)
- **Cohesion**: 1.0 (stay in safe areas)
- **Predator Avoidance**: 3.0 (highest priority - flee!)
- **Border Avoidance**: 2.0 (avoid edges)

## Energy-Based Hunting System (Cheetah Mode)

The energy-based predator simulates realistic animal behavior like a cheetah:

### Energy System:
- **Max Energy**: 100% (starts full)
- **Energy Drain**: 1.5 units/second while moving (reduced for better hunting)
- **Energy Recovery**: 2.0 units/second while resting (increased)
- **Sprint Cost**: +4.0 extra energy/second when sprinting (reduced)
- **Minimum Energy to Hunt**: 20% (must rest when below this, reduced)

### Speed Modes:
- **Stalking Speed**: 0.8 m/s (faster stalking for better hunting)
- **Hunting Speed**: 1.8 m/s (faster hunting speed)
- **Sprint Speed**: 3.5 m/s (very fast sprint, high energy cost)

### Behavior Modes:
- **Rest Mode**: When energy < 20%, predator stops and recovers energy
- **Stalking Mode**: Faster movement when far from prey (5.5 units)
- **Hunting Mode**: Fast speed when within stalking range
- **Sprint Mode**: Fast attack when close to prey (2.0 units) AND energy > 30%
- **Patrol Mode**: Energy-efficient movement when no prey detected

### Realistic Features:
- **Energy Management**: Must balance hunting vs resting
- **Sprint Decision**: Only sprints when close to prey AND has enough energy
- **Recovery Periods**: Must rest when energy is low
- **Efficient Movement**: Uses slower speeds to conserve energy

## Next Steps

The system now has a complete prey-predator simulation with both stalking and energy-based hunting behaviors. You can experiment with different hunting strategies from the list below.

## Hunting Logic Ideas (Future Implementations)

### 1. Simple Distance-Based Hunting
- Predator moves toward prey when within detection range
- Simple distance calculation and direct movement
- Easy to implement and test

### 2. Line-of-Sight Hunting
- Predator can only "see" prey if there's a clear path
- Check if prey is within a cone of vision in front of predator
- More realistic hunting behavior

### 3. Predictive Hunting
- Predator predicts where prey will be and moves to intercept
- Calculate prey's velocity and direction, move to intercept point
- More intelligent, harder for prey to escape

### 4. Stalking Behavior (Current Implementation)
- Predator moves slowly when far from prey, speeds up when close
- Different speeds based on distance to prey
- Realistic hunting behavior with speed variations

### 5. Territory-Based Hunting
- Predator patrols specific areas and hunts prey that enters its territory
- Define hunting zones, only hunt when prey enters zone
- Strategic behavior with defined territories

### 6. Energy-Based Hunting
- Predator has limited "energy" and must balance hunting vs resting
- Energy decreases while moving, increases while stationary
- Realistic constraints with energy management

### 7. Multi-Mode Hunting
- Predator switches between different hunting modes
- Patrolling → Detecting → Pursuing → Attacking
- Complex, interesting behavior with state machines 