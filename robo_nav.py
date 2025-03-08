import cv2
import numpy as np
import math
import serial
import time
from collections import deque

# Initialize serial communication (Update COM port for your system)
arduino = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)

# HSV bounds for black color
lower_black = np.array([0, 0, 0])
upper_black = np.array([180, 255, 100])

# Define rectangle dimensions (adjust as needed)
rect_width = 500
rect_height = 250

# Load ArUco dictionary and detector
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
aruco_params = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

# Navigation graph and directional mapping
graph = {
    0: {"north": 1},
    1: {"south": 0, "north": 4, "east": 2, "west": 3},
    2: {"west": 1},
    3: {"east": 1},
    4: {"south": 1, "east": 5, "west": 6},
    5: {"south": 11, "north": 10, "east": 9, "west": 4},
    6: {"south": 7, "east": 4, "west": 8},
    7: {"east": 6},
    8: {"south": 9},
    9: {"west": 5},
    10: {"south": 5},
    11: {"north": 5},
}

direction_map = {
    "north": {"left": "west", "right": "east", "reverse": "south"},
    "east": {"left": "north", "right": "south", "reverse": "west"},
    "south": {"left": "east", "right": "west", "reverse": "north"},
    "west": {"left": "south", "right": "north", "reverse": "east"},
}

def find_directions(current, target):
    if current == target:
        return []
    visited = set()
    queue = deque([(current, [])])
    visited.add(current)
    while queue:
        current_node, path = queue.popleft()
        if current_node == target:
            return path
        for direction, neighbor in graph[current_node].items():
            if neighbor not in visited:
                visited.add(neighbor)
                new_path = path + [direction]
                queue.append((neighbor, new_path))
    return None

# NEW: Compute one effective turning command per platform transition.
def get_effective_turn_instructions(platforms, initial_facing="north"):
    instructions = []
    current_facing = initial_facing
    # For each segment from one marker to the next...
    for i in range(1, len(platforms)):
        path = find_directions(platforms[i-1], platforms[i])
        if path is None:
            raise ValueError(f"Cannot reach platform {platforms[i]} from {platforms[i-1]}")
        final_direction = path[-1]  # Net required direction for the segment
        # Determine the effective turn from current_facing to final_direction.
        if final_direction == current_facing:
            instruction = "straight"
        elif direction_map[current_facing]["left"] == final_direction:
            instruction = "left"
        elif direction_map[current_facing]["right"] == final_direction:
            instruction = "right"
        elif direction_map[current_facing]["reverse"] == final_direction:
            instruction = "rotate 180"
        else:
            raise ValueError(f"Invalid turn from {current_facing} to {final_direction}")
        instructions.append(instruction)
        current_facing = final_direction
    return instructions

def pid_control(error, Kp=2.0, Ki=0.01, Kd=5.0):
    pid_control.prev_error = getattr(pid_control, 'prev_error', 0)
    pid_control.integral = getattr(pid_control, 'integral', 0)
    pid_control.integral += error
    derivative = error - pid_control.prev_error
    output = Kp * error + Ki * pid_control.integral + Kd * derivative
    pid_control.prev_error = error
    return max(5, min(int(abs(output)), 50))

def send_motor_command(command):
    arduino.write((command + '\n').encode())
    print(f"Sent: {command}")

def detect_red_color(frame, roi):
    x, y, w, h = roi
    roi_frame = frame[y:y+h, x:x+w]
    hsv = cv2.cvtColor(roi_frame, cv2.COLOR_BGR2HSV)
    # Two ranges for red
    lower_red1 = np.array([0, 70, 50])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([170, 70, 50])
    upper_red2 = np.array([180, 255, 255])
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    red_mask = cv2.bitwise_or(mask1, mask2)
    red_area = cv2.countNonZero(red_mask)
    threshold = 500  # Adjust threshold as needed
    return red_area > threshold

def detect_black_line(frame, roi, aruco_bboxes):
    x, y, w, h = roi
    roi_frame = frame[y:y+h, x:x+w]
    hsv = cv2.cvtColor(roi_frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower_black, upper_black)
    
    # Remove regions corresponding to ArUco markers within the ROI.
    for bbox in aruco_bboxes:
        bx_min, by_min, bx_max, by_max = bbox
        if bx_min >= x and by_min >= y and bx_max <= x + w and by_max <= y + h:
            cv2.rectangle(mask, (bx_min - x, by_min - y), (bx_max - x, by_max - y), (0, 0, 0), -1)
    
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        max_contour = max(contours, key=cv2.contourArea)
        M = cv2.moments(max_contour)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"]) + x
            cy = int(M["m01"] / M["m00"]) + y
            return cx, cy, max_contour, mask
    return None, None, None, mask

def measure_thickness(mask):
    dist_transform = cv2.distanceTransform(mask, cv2.DIST_L2, 5)
    max_dist = np.max(dist_transform)
    return max_dist * 2, dist_transform

def detect_aruco_markers(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = detector.detectMarkers(gray)
    aruco_bboxes = []
    detected_ids = []
    
    if ids is not None:
        for i in range(len(ids)):
            marker_id = int(ids[i][0])
            detected_ids.append(marker_id)
            x_min = int(np.min(corners[i][0][:, 0]))
            y_min = int(np.min(corners[i][0][:, 1]))
            x_max = int(np.max(corners[i][0][:, 0]))
            y_max = int(np.max(corners[i][0][:, 1]))
            aruco_bboxes.append((x_min, y_min, x_max, y_max))
            cv2.rectangle(frame, (x_min, y_min), (x_max, y_max), (0, 255, 255), 2)
    return aruco_bboxes, detected_ids

TURN_DURATION = 2.0

def show_live_camera_feed():
    global current_marker_index, current_state
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Error: Could not open camera.")
        return
    
    frame_count = 0
    start_time = time.time()
    red_command_sent = False

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: Failed to capture frame.")
            break

        frame_count += 1
        elapsed_time = time.time() - start_time
        fps = frame_count / elapsed_time if elapsed_time > 0 else 0
        cv2.putText(frame, f"FPS: {fps:.2f}", (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
        
        height, width, _ = frame.shape
        center_x, center_y = width // 2, height // 2
        rect_x, rect_y = center_x - rect_width // 2, center_y - rect_height // 2
        cv2.rectangle(frame, (rect_x, rect_y), (rect_x + rect_width, rect_y + rect_height), (255, 0, 0), 2)
        
        aruco_bboxes, detected_ids = detect_aruco_markers(frame)
        black_x, black_y, black_contour, mask = detect_black_line(frame, (rect_x, rect_y, rect_width, rect_height), aruco_bboxes)
        thickness, _ = measure_thickness(mask)

        if thickness is not None:
            text = f"Thickness: {thickness:.2f} px"
            text_size = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 1, 2)[0]
            text_x = frame.shape[1] - text_size[0] - 10
            text_y = 30
            cv2.putText(frame, text, (text_x, text_y), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2, cv2.LINE_AA)

        if current_state == "line_following":
            # Check for red detection (for any special event)
            if detect_red_color(frame, (rect_x, rect_y, rect_width, rect_height)):
                if not red_command_sent:
                    send_motor_command("3:n")  # Example command when red is detected.
                    red_command_sent = True
            else:
                red_command_sent = False

            if black_contour is not None:
                cv2.drawContours(frame, [black_contour + np.array([rect_x, rect_y])], -1, (0, 255, 0), 2)
                cv2.circle(frame, (black_x, black_y), 5, (0, 255, 0), -1)
                bottom_x, bottom_y = center_x, rect_y + rect_height
                cv2.line(frame, (black_x, black_y), (bottom_x, bottom_y), (0, 255, 255), 2)
                
                delta_x, delta_y = bottom_x - black_x, bottom_y - black_y
                angle = math.degrees(math.atan2(delta_y, delta_x))
                cv2.putText(frame, f"Angle: {angle:.2f}°", (10, 70),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2, cv2.LINE_AA)
                error = 90 - angle
                pulses = pid_control(error)
                
                if 60 <= thickness <= 100:
                    if abs(error) < 40:
                        send_motor_command(f"3:{pulses}")
                    elif error > 0:
                        send_motor_command(f"2:{pulses}")
                    else:
                        send_motor_command(f"1:{pulses}")

            # Check for navigation markers
            for marker_id in detected_ids:
                # Use current_marker_index to decide the next expected marker.
                if marker_id == platform_path[current_marker_index]:
                    # If final marker reached, stop.
                    if current_marker_index == len(platform_path) - 1:
                        send_motor_command("3:0")
                        current_state = "halted"
                        print(f"Reached target marker {platform_path[-1]}")
                        break
                    else:
                        # Execute the precomputed turn for this segment.
                        turn_instruction = instructions[current_marker_index - 1]
                        current_state = "turning"
                        turn_start_time = time.time()
                        if turn_instruction == "left":
                            send_motor_command("1:40")
                        elif turn_instruction == "right":
                            send_motor_command("2:40")
                        elif turn_instruction == "straight":
                            send_motor_command("3:10")
                        elif turn_instruction == "rotate 180":
                            send_motor_command("6:45")
                        current_marker_index += 1
                    break

        elif current_state == "turning":
            # # Continue turning until a proper black line is detected.
            # # black_x, black_y, black_contour, mask = detect_black_line(frame, (rect_x, rect_y, rect_width, rect_height), aruco_bboxes)
            # # thickness, _ = measure_thickness(mask)
            # if black_contour is not None and thickness is not None and 60 <= thickness <= 100:
            #     send_motor_command("3:0")
            #     current_state = "line_following"
            if time.time() - turn_start_time >= TURN_DURATION:
                send_motor_command("3:0")  # Stop turning after fixed duration
                current_state = "line_following"

        cv2.imshow('Object Detection', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    cap.release()
    cv2.destroyAllWindows()

# Global navigation variables
platform_path = [0, 3, 2]  # Start at marker 0, then go to marker 3, then marker 2.
initial_facing = "north"
# Compute one turning command per transition
instructions = get_effective_turn_instructions(platform_path, initial_facing)
print(f"Platform path: {platform_path}")
print(f"Turn instructions: {instructions}")

# Initialize marker index to 1 (since we start at the first marker)
current_marker_index = 1  
current_state = "line_following"

if not platform_path:
    print("Platform path is empty!")
else:
    show_live_camera_feed()
