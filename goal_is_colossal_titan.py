import sensor
import time
import math

# Yellow and blue color tracking thresholds (L Min, L Max, A Min, A Max, B Min, B Max)
yellow_threshold = (50, 100, -10, 30, 30, 127)
blue_threshold = (0, 100, -80, 0, -128, -30)

# Global variables for enemy and home goal colors
enemy_goal_color = None  # Will store the color of the enemy goal
home_goal_color = None   # Will store the color of the home goal
initial_detection_done = False

# Constants for distance calculation
GOAL_ACTUAL_WIDTH_CM = 43  # Width of the goal post in cm (adjust to actual value)
GOAL_ACTUAL_HEIGHT_CM = 16  # Height of the goal post in cm (adjust to actual value)

# Properly calibrated focal length based on test results
FOCAL_LENGTH = 300  # Based on calibration data

# Distance filtering
distance_buffer = {}  # Dictionary to store distance buffers for different objects
BUFFER_SIZE = 5      # Number of frames to average over
MIN_BLOB_SIZE = 100  # Minimum blob size to consider

# Step 1: Initialize the camera
sensor.reset()
sensor.set_pixformat(sensor.RGB565)  # Set pixel format
sensor.set_framesize(sensor.QVGA)  # Set frame size
sensor.skip_frames(time=2000)  # Let the camera initialize

# Step 2: Lock exposure
sensor.set_auto_exposure(True)  # Enable auto-exposure temporarily
sensor.skip_frames(time=2000)  # Allow time for adjustment
sensor.set_auto_exposure(False)  # Lock the current exposure
print("Auto-exposure locked.")

# Disable auto gain and auto white balance for consistent color tracking
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)

clock = time.clock()

# Modify the calculate_distance function to only use height
def calculate_distance(height_pixels):
    """Calculate distance based only on apparent height"""
    # More reliable when goal is partially obstructed
    return (GOAL_ACTUAL_HEIGHT_CM * FOCAL_LENGTH) / height_pixels

# Add a new function to calculate visible width
def calculate_visible_width(blob_width, distance_cm):
    """Calculate the actual width of the visible portion of the goal in cm"""
    visible_width_cm = (blob_width * distance_cm) / FOCAL_LENGTH
    return visible_width_cm

def get_filtered_distance(new_distance, object_id):
    """Apply a simple moving average filter to distance measurements"""
    global distance_buffer

    # Initialize buffer for this object if it doesn't exist
    if object_id not in distance_buffer:
        distance_buffer[object_id] = []

    # Add new measurement to buffer
    distance_buffer[object_id].append(new_distance)

    # Keep buffer at desired size
    if len(distance_buffer[object_id]) > BUFFER_SIZE:
        distance_buffer[object_id].pop(0)

    # Return the average
    return sum(distance_buffer[object_id]) / len(distance_buffer[object_id])

# Step 3: Main loop to detect the enemy goal color and blobs
while True:
    clock.tick()
    img = sensor.snapshot()

    # Initial enemy goal detection
    if not initial_detection_done:
        # Check for yellow goal
        yellow_stats = img.get_statistics(thresholds=[yellow_threshold])
        blue_stats = img.get_statistics(thresholds=[blue_threshold])

        yellow_pixels = yellow_stats.mean()
        blue_pixels = blue_stats.mean()

        print(f"Yellow pixels mean: {yellow_pixels}")
        print(f"Blue pixels mean: {blue_pixels}")

        # Determine the dominant color
        if yellow_pixels > blue_pixels:
            enemy_goal_color = yellow_threshold
            home_goal_color = blue_threshold
            enemy_goal_color_name = "Yellow"
            home_goal_color_name = "Blue"
        else:
            enemy_goal_color = blue_threshold
            home_goal_color = yellow_threshold
            enemy_goal_color_name = "Blue"
            home_goal_color_name = "Yellow"

        initial_detection_done = True
        print(f"Enemy goal color detected: {enemy_goal_color_name}")
    else:
        # Get all blobs with sufficient size
        blobs = img.find_blobs([enemy_goal_color, home_goal_color],
                               pixels_threshold=MIN_BLOB_SIZE,
                               area_threshold=MIN_BLOB_SIZE*2)

        # Find the largest blob of each type (most likely to be the actual goal)
        largest_enemy_blob = None
        largest_home_blob = None

        for blob in blobs:
            # Calculate blob area
            area = blob.w() * blob.h()

            if blob.code() == 1:  # Enemy goal blob
                if largest_enemy_blob is None or area > largest_enemy_blob.w() * largest_enemy_blob.h():
                    largest_enemy_blob = blob
            else:  # Home goal blob
                if largest_home_blob is None or area > largest_home_blob.w() * largest_home_blob.h():
                    largest_home_blob = blob

        # Process the largest enemy blob if found
        if largest_enemy_blob:
            # Calculate distance using only height
            blob_height = largest_enemy_blob.h()
            raw_distance = calculate_distance(blob_height)
            distance_cm = get_filtered_distance(raw_distance, "enemy")

            # Calculate visible width based on the distance
            blob_width = largest_enemy_blob.w()
            visible_width_cm = calculate_visible_width(blob_width, distance_cm)

            # Draw visual indicators
            color = (255, 0, 0)  # Red color for enemy blobs
            label = f"Enemy: {distance_cm:.1f}cm, W:{visible_width_cm:.1f}cm"

            img.draw_rectangle(largest_enemy_blob.rect(), color=color)
            img.draw_cross(largest_enemy_blob.cx(), largest_enemy_blob.cy(), color=color)
            img.draw_keypoints(
                [(largest_enemy_blob.cx(), largest_enemy_blob.cy(),
                  int(math.degrees(largest_enemy_blob.rotation())))],
                size=20,
                color=color
            )

            # Add text label with distance information
            img.draw_string(largest_enemy_blob.cx(), largest_enemy_blob.cy() - 20,
                           label, color=color)

            # Print distance information to console
            print(f"Enemy Goal: {distance_cm:.1f}cm at ({largest_enemy_blob.cx()}, {largest_enemy_blob.cy()})")

        # Process the largest home blob if found
        if largest_home_blob:
            # Calculate distance using only height
            blob_height = largest_home_blob.h()
            raw_distance = calculate_distance(blob_height)
            distance_cm = get_filtered_distance(raw_distance, "home")

            # Calculate visible width based on the distance
            blob_width = largest_home_blob.w()
            visible_width_cm = calculate_visible_width(blob_width, distance_cm)

            # Draw visual indicators
            color = (0, 255, 0)  # Green color for home blobs
            label = f"Home: {distance_cm:.1f}cm, W:{visible_width_cm:.1f}cm"

            img.draw_rectangle(largest_home_blob.rect(), color=color)
            img.draw_cross(largest_home_blob.cx(), largest_home_blob.cy(), color=color)
            img.draw_keypoints(
                [(largest_home_blob.cx(), largest_home_blob.cy(),
                  int(math.degrees(largest_home_blob.rotation())))],
                size=20,
                color=color
            )

            # Add text label with distance information
            img.draw_string(largest_home_blob.cx(), largest_home_blob.cy() - 20,
                           label, color=color)

            # Print distance information to console
            print(f"Home Goal: {distance_cm:.1f}cm at ({largest_home_blob.cx()}, {largest_home_blob.cy()})")

    print(f"FPS: {clock.fps()}")
