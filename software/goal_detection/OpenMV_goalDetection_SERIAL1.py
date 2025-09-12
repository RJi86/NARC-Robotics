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

# Initial detection variables
DETECTION_BUFFER_TIME = 3000  # 3 seconds in milliseconds
detection_start_time = None
yellow_distance_measurements = []
blue_distance_measurements = []
enemy_goal_color_name = None
home_goal_color_name = None

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
    if height_pixels <= 0:
        return float('inf')  # Return infinity for invalid measurements
    # More reliable when goal is partially obstructed
    return (GOAL_ACTUAL_HEIGHT_CM * FOCAL_LENGTH) / height_pixels

def get_filtered_distance(new_distance, object_id):
    """Apply a simple moving average filter to distance measurements"""
    global distance_buffer

    # Skip invalid measurements
    if new_distance <= 0 or new_distance == float('inf'):
        return None

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

def find_largest_blob_by_color(blobs, color_threshold):
    """Find the largest blob matching a specific color threshold"""
    largest_blob = None
    largest_area = 0
    
    for blob in blobs:
        # Check if this blob matches our color (code 1 for first threshold, code 2 for second)
        blob_matches = False
        if color_threshold == yellow_threshold and blob.code() == 1:
            blob_matches = True
        elif color_threshold == blue_threshold and blob.code() == 2:
            blob_matches = True
        
        if blob_matches:
            area = blob.w() * blob.h()
            if area > largest_area:
                largest_area = area
                largest_blob = blob
    
    return largest_blob

def safe_frame_operation():
    """Safely capture a frame with error handling"""
    try:
        return sensor.snapshot()
    except Exception as e:
        print(f"Frame capture error: {e}")
        # Wait a bit and try again
        time.sleep(10)  # 10ms delay
        try:
            return sensor.snapshot()
        except Exception as e:
            print(f"Second frame capture failed: {e}")
            return None

def send_goal_data_to_arduino(goal_type, distance_cm, height_pixels, x_pos, y_pos):
    """Send goal detection data to Arduino via UART"""
    try:
        # Format: TYPE,distance,height,x,y\n
        # TYPE is 'E' for enemy, 'H' for home
        data_string = f"{goal_type},{distance_cm:.1f},{height_pixels},{x_pos},{y_pos}\n"
        print(data_string, end='')  # Send to UART (Arduino)
        # Also print to console for debugging
        print(f"Sent to Arduino: {data_string.strip()}")
    except Exception as e:
        print(f"Error sending data to Arduino: {e}")

# Step 3: Main loop to detect the enemy goal color and blobs
while True:
    try:
        clock.tick()
        img = safe_frame_operation()
        
        if img is None:
            continue  # Skip this iteration if frame capture failed
        
        # Initial enemy goal detection with 3-second buffer
        if not initial_detection_done:
            current_time = time.ticks_ms()
            
            # Start the detection timer on first run
            if detection_start_time is None:
                detection_start_time = current_time
                print("Starting 3-second goal detection phase...")
                print("Keep camera steady and ensure both goals are visible!")
            
            # Check if we're still in the 3-second detection window
            if time.ticks_diff(current_time, detection_start_time) < DETECTION_BUFFER_TIME:
                # Find blobs for both colors
                all_blobs = img.find_blobs([yellow_threshold, blue_threshold],
                                         pixels_threshold=MIN_BLOB_SIZE,
                                         area_threshold=MIN_BLOB_SIZE*2)
                
                # Find largest blob of each color
                yellow_blob = find_largest_blob_by_color(all_blobs, yellow_threshold)
                blue_blob = find_largest_blob_by_color(all_blobs, blue_threshold)
                
                # Measure distances if blobs are found
                if yellow_blob and yellow_blob.h() > 0:
                    yellow_distance = calculate_distance(yellow_blob.h())
                    if yellow_distance != float('inf'):
                        yellow_distance_measurements.append(yellow_distance)
                        # Draw yellow blob during detection
                        img.draw_rectangle(yellow_blob.rect(), color=(255, 255, 0))
                        img.draw_string(yellow_blob.cx(), yellow_blob.cy() - 20,
                                      f"Y: {yellow_distance:.1f}cm", color=(255, 255, 0))
                
                if blue_blob and blue_blob.h() > 0:
                    blue_distance = calculate_distance(blue_blob.h())
                    if blue_distance != float('inf'):
                        blue_distance_measurements.append(blue_distance)
                        # Draw blue blob during detection
                        img.draw_rectangle(blue_blob.rect(), color=(0, 0, 255))
                        img.draw_string(blue_blob.cx(), blue_blob.cy() - 20,
                                      f"B: {blue_distance:.1f}cm", color=(0, 0, 255))
                
                # Show countdown
                remaining_time = (DETECTION_BUFFER_TIME - time.ticks_diff(current_time, detection_start_time)) / 1000.0
                img.draw_string(10, 10, f"Detection: {remaining_time:.1f}s", color=(255, 255, 255))
                
            else:
                # Detection period is over, analyze results
                if len(yellow_distance_measurements) > 0 and len(blue_distance_measurements) > 0:
                    # Calculate average distances
                    avg_yellow_distance = sum(yellow_distance_measurements) / len(yellow_distance_measurements)
                    avg_blue_distance = sum(blue_distance_measurements) / len(blue_distance_measurements)
                    
                    print(f"Average Yellow distance: {avg_yellow_distance:.1f}cm")
                    print(f"Average Blue distance: {avg_blue_distance:.1f}cm")
                    
                    # The further goal is the enemy goal
                    if avg_yellow_distance > avg_blue_distance:
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
                    print(f"Enemy goal color determined: {enemy_goal_color_name} (further away)")
                    print(f"Home goal color determined: {home_goal_color_name} (closer)")
                    
                else:
                    # Reset detection if we didn't get good measurements
                    print("Insufficient measurements, restarting detection...")
                    detection_start_time = None
                    yellow_distance_measurements.clear()
                    blue_distance_measurements.clear()
                    
        else:
            # Normal operation after initial detection
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
            if largest_enemy_blob and largest_enemy_blob.h() > 0:
                # Calculate distance using only height
                blob_height = largest_enemy_blob.h()
                raw_distance = calculate_distance(blob_height)
                distance_cm = get_filtered_distance(raw_distance, "enemy")
                
                if distance_cm is not None:
                    # Draw visual indicators
                    color = (255, 0, 0)  # Red color for enemy blobs
                    label = f"Enemy: {distance_cm:.1f}cm"

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

                    # Send data to Arduino
                    send_goal_data_to_arduino('E', distance_cm, blob_height, 
                                            largest_enemy_blob.cx(), largest_enemy_blob.cy())

            # Process the largest home blob if found
            if largest_home_blob and largest_home_blob.h() > 0:
                # Calculate distance using only height
                blob_height = largest_home_blob.h()
                raw_distance = calculate_distance(blob_height)
                distance_cm = get_filtered_distance(raw_distance, "home")
                
                if distance_cm is not None:
                    # Draw visual indicators
                    color = (0, 255, 0)  # Green color for home blobs
                    label = f"Home: {distance_cm:.1f}cm"

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

                    # Send data to Arduino
                    send_goal_data_to_arduino('H', distance_cm, blob_height, 
                                            largest_home_blob.cx(), largest_home_blob.cy())

        print(f"FPS: {clock.fps()}")
        
    except Exception as e:
        print(f"Main loop error: {e}")
        # Add a small delay to prevent rapid error loops
        time.sleep(50)  # 50ms delay
        continue