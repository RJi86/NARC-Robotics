import sensor
import time
import math
from machine import I2C, Pin

# I2C Communication setup
i2c = I2C(2, scl=Pin('P4'), sda=Pin('P5'), freq=100000)
ARDUINO_ADDRESS = 0x08  # Arduino I2C address

# Yellow and blue color tracking thresholds
yellow_threshold = (50, 100, -10, 30, 30, 127)
blue_threshold = (0, 100, -80, 0, -128, -30)

# Global variables for enemy and home goal colors
enemy_goal_color = None
home_goal_color = None
initial_detection_done = False

# Constants for distance calculation
GOAL_ACTUAL_HEIGHT_CM = 16
FOCAL_LENGTH = 300

# Distance filtering
distance_buffer = {}
BUFFER_SIZE = 5
MIN_BLOB_SIZE = 100

# Initial detection variables
DETECTION_BUFFER_TIME = 3000  # 3 seconds
detection_start_time = None
yellow_distance_measurements = []
blue_distance_measurements = []
enemy_goal_color_name = None
home_goal_color_name = None

# Initialize camera
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)

# Lock exposure settings
sensor.set_auto_exposure(True)
sensor.skip_frames(time=2000)
sensor.set_auto_exposure(False)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)

clock = time.clock()

def calculate_distance(height_pixels):
    """Calculate distance based on apparent height"""
    if height_pixels <= 0:
        return float('inf')
    return (GOAL_ACTUAL_HEIGHT_CM * FOCAL_LENGTH) / height_pixels

def get_filtered_distance(new_distance, object_id):
    """Apply moving average filter to distance measurements"""
    global distance_buffer
    
    if new_distance <= 0 or new_distance == float('inf'):
        return None
    
    if object_id not in distance_buffer:
        distance_buffer[object_id] = []
    
    distance_buffer[object_id].append(new_distance)
    
    if len(distance_buffer[object_id]) > BUFFER_SIZE:
        distance_buffer[object_id].pop(0)
    
    return sum(distance_buffer[object_id]) / len(distance_buffer[object_id])

def find_largest_blob_by_color(blobs, color_threshold):
    """Find the largest blob matching a specific color threshold"""
    largest_blob = None
    largest_area = 0
    
    for blob in blobs:
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

def send_goal_data_to_arduino(goal_type, distance, x, y):
    """Send goal data to Arduino via I2C"""
    try:
        # Format: [goal_type, distance_high, distance_low, x_high, x_low, y_high, y_low]
        distance_int = int(distance * 10)  # Send distance with 1 decimal precision
        
        data = bytearray([
            ord(goal_type),  # 'E' for enemy, 'H' for home
            (distance_int >> 8) & 0xFF,  # High byte of distance
            distance_int & 0xFF,         # Low byte of distance
            (x >> 8) & 0xFF,             # High byte of x
            x & 0xFF,                    # Low byte of x
            (y >> 8) & 0xFF,             # High byte of y
            y & 0xFF                     # Low byte of y
        ])
        
        i2c.writeto(ARDUINO_ADDRESS, data)
        print(f"Sent {goal_type}: {distance:.1f}cm at ({x},{y})")
        
    except Exception as e:
        print(f"I2C communication error: {e}")

def safe_frame_operation():
    """Safely capture a frame with error handling"""
    try:
        return sensor.snapshot()
    except Exception as e:
        print(f"Frame capture error: {e}")
        time.sleep(10)
        try:
            return sensor.snapshot()
        except Exception as e:
            print(f"Second frame capture failed: {e}")
            return None

# Main loop
while True:
    try:
        clock.tick()
        img = safe_frame_operation()
        
        if img is None:
            continue
        
        # Initial detection phase with 3-second buffer
        if not initial_detection_done:
            current_time = time.ticks_ms()
            
            if detection_start_time is None:
                detection_start_time = current_time
                print("Starting 3-second goal detection phase...")
            
            if time.ticks_diff(current_time, detection_start_time) < DETECTION_BUFFER_TIME:
                # Find blobs for both colors
                all_blobs = img.find_blobs([yellow_threshold, blue_threshold],
                                         pixels_threshold=MIN_BLOB_SIZE,
                                         area_threshold=MIN_BLOB_SIZE*2)
                
                yellow_blob = find_largest_blob_by_color(all_blobs, yellow_threshold)
                blue_blob = find_largest_blob_by_color(all_blobs, blue_threshold)
                
                # Measure distances
                if yellow_blob and yellow_blob.h() > 0:
                    yellow_distance = calculate_distance(yellow_blob.h())
                    if yellow_distance != float('inf'):
                        yellow_distance_measurements.append(yellow_distance)
                        img.draw_rectangle(yellow_blob.rect(), color=(255, 255, 0))
                        img.draw_string(yellow_blob.cx(), yellow_blob.cy() - 20,
                                      f"Y: {yellow_distance:.1f}cm", color=(255, 255, 0))
                
                if blue_blob and blue_blob.h() > 0:
                    blue_distance = calculate_distance(blue_blob.h())
                    if blue_distance != float('inf'):
                        blue_distance_measurements.append(blue_distance)
                        img.draw_rectangle(blue_blob.rect(), color=(0, 0, 255))
                        img.draw_string(blue_blob.cx(), blue_blob.cy() - 20,
                                      f"B: {blue_distance:.1f}cm", color=(0, 0, 255))
                
                # Show countdown
                remaining_time = (DETECTION_BUFFER_TIME - time.ticks_diff(current_time, detection_start_time)) / 1000.0
                img.draw_string(10, 10, f"Detection: {remaining_time:.1f}s", color=(255, 255, 255))
                
            else:
                # Detection period complete
                if len(yellow_distance_measurements) > 0 and len(blue_distance_measurements) > 0:
                    avg_yellow_distance = sum(yellow_distance_measurements) / len(yellow_distance_measurements)
                    avg_blue_distance = sum(blue_distance_measurements) / len(blue_distance_measurements)
                    
                    print(f"Average Yellow distance: {avg_yellow_distance:.1f}cm")
                    print(f"Average Blue distance: {avg_blue_distance:.1f}cm")
                    
                    # Further goal is enemy, closer is home
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
                    print(f"Enemy goal: {enemy_goal_color_name} (further)")
                    print(f"Home goal: {home_goal_color_name} (closer)")
                    
                else:
                    print("Insufficient measurements, restarting...")
                    detection_start_time = None
                    yellow_distance_measurements.clear()
                    blue_distance_measurements.clear()
                    
        else:
            # Normal operation after detection
            blobs = img.find_blobs([enemy_goal_color, home_goal_color],
                                   pixels_threshold=MIN_BLOB_SIZE,
                                   area_threshold=MIN_BLOB_SIZE*2)

            largest_enemy_blob = None
            largest_home_blob = None

            for blob in blobs:
                area = blob.w() * blob.h()
                if blob.code() == 1:  # Enemy goal
                    if largest_enemy_blob is None or area > largest_enemy_blob.w() * largest_enemy_blob.h():
                        largest_enemy_blob = blob
                else:  # Home goal
                    if largest_home_blob is None or area > largest_home_blob.w() * largest_home_blob.h():
                        largest_home_blob = blob

            # Process enemy goal
            if largest_enemy_blob and largest_enemy_blob.h() > 0:
                raw_distance = calculate_distance(largest_enemy_blob.h())
                distance_cm = get_filtered_distance(raw_distance, "enemy")
                
                if distance_cm is not None:
                    color = (255, 0, 0)  # Red for enemy
                    label = f"Enemy: {distance_cm:.1f}cm"
                    
                    img.draw_rectangle(largest_enemy_blob.rect(), color=color)
                    img.draw_cross(largest_enemy_blob.cx(), largest_enemy_blob.cy(), color=color)
                    img.draw_string(largest_enemy_blob.cx(), largest_enemy_blob.cy() - 20,
                                   label, color=color)
                    
                    # Send to Arduino
                    send_goal_data_to_arduino('E', distance_cm, 
                                            largest_enemy_blob.cx(), 
                                            largest_enemy_blob.cy())

            # Process home goal
            if largest_home_blob and largest_home_blob.h() > 0:
                raw_distance = calculate_distance(largest_home_blob.h())
                distance_cm = get_filtered_distance(raw_distance, "home")
                
                if distance_cm is not None:
                    color = (0, 255, 0)  # Green for home
                    label = f"Home: {distance_cm:.1f}cm"
                    
                    img.draw_rectangle(largest_home_blob.rect(), color=color)
                    img.draw_cross(largest_home_blob.cx(), largest_home_blob.cy(), color=color)
                    img.draw_string(largest_home_blob.cx(), largest_home_blob.cy() - 20,
                                   label, color=color)
                    
                    # Send to Arduino
                    send_goal_data_to_arduino('H', distance_cm, 
                                            largest_home_blob.cx(), 
                                            largest_home_blob.cy())

        print(f"FPS: {clock.fps()}")
        
    except Exception as e:
        print(f"Main loop error: {e}")
        time.sleep(50)
        continue