- Need to change the pins for motors

## Key States

- STOPPED: Robot is inactive, waiting for button press
- FIND_BALL: Rotating to locate the ball
- APPROACH_BALL: Moving toward the ball while keeping goal in view
- ALIGN_FOR_CATCH: Carefully positioning to catch the ball
- DRIBBLE_TOWARD_GOAL: Moving toward goal with captured ball
- AVOID_BOUNDARY: Temporarily moving away from field boundaries

## The Robot's Strategy

### Finding the Ball:
- Rotates to search for ball
- If it sees the goal, it rotates in a way that keeps the goal in view

### Approaching the Ball:
- Moves directly toward the ball using vector math
- Adjusts speed based on distance (slower when far, faster when close)
- Tries to rotate slightly to keep enemy goal in front camera view

### Aligning to Catch:
- When close to ball, carefully positions to get ball at 0° angle
- Makes small adjustments to keep enemy goal in front view

### Dribbling Toward Goal:
- Once ball is caught, locates enemy goal
- Uses combined movement and rotation to move straight at goal
- Keeps the goal centered in view

### Avoiding Boundaries:
- Constantly checks LDR sensors for white boundaries
- If detected, immediately moves away in opposite direction
- Returns to previous activity after moving away

### Button Control:
- Press once: Stop robot
- Press again: Resume previous activity

## Smart Features

Vector-Based Movement: Uses matrix math for precise omnidirectional movement
Goal-Aware Navigation: Always tries to keep enemy goal in front camera
Smoothed Readings: Averages sensor values to prevent jittery movement
Dynamic Speed Control: Adjusts speed based on confidence and situation
Combined Movement & Rotation: Can move and rotate simultaneously for efficient positioning