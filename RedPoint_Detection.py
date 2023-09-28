# First import the library
import pyrealsense2 as rs
# Import Numpy for easy array manipulation
import numpy as np
# Import OpenCV for easy image rendering
import cv2
# Import datetime module
from datetime import datetime  
# Import os to stop
import os
import math

# determine the minimum pixel in the circle to be detect
cir_pixel = 30

def initialize_pipeline():
    # Create a pipeline
    pipeline = rs.pipeline()
    # Create a config and configure the pipeline to stream
    config = rs.config()

    # Get device product line for setting a supporting resolution
    pipeline_wrapper = rs.pipeline_wrapper(pipeline)
    pipeline_profile = config.resolve(pipeline_wrapper)

    # Get the information of the device
    device = pipeline_profile.get_device()
    device_product_line = str(device.get_info(rs.camera_info.product_line))

    found_rgb = False
    for s in device.sensors:
        if s.get_info(rs.camera_info.name) == 'RGB Camera':
            found_rgb = True
            break
    if not found_rgb:
        print("The demo requires Depth camera with Color sensor")
        exit(0)

    # Set FPS for stream
    FPS_setting = 30
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, FPS_setting) # (stream type,resolution, image format, fps)

    if device_product_line == 'L500':
        config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, FPS_setting)
    else:
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, FPS_setting)

    return pipeline, config

def get_depth_scale(profile):
    depth_sensor = profile.get_device().first_depth_sensor()
    return depth_sensor.get_depth_scale()

# rs.align allows us to perform alignment of depth frames to others frames
# The "align_to" is the stream type to which we plan to align depth frames
def initialize_align():
    align_to = rs.stream.color
    return rs.align(align_to)

def create_timestamped_filename():
    current_datetime = datetime.now()
    timestamp = current_datetime.strftime("%Y%m%d_%H%M%S") # Format: YYYYMMDD_HHMMSS
    return f'captured_points_{timestamp}.txt'

def save_coordinates_to_file(filename, captured_coordinates):
    with open(filename, 'a') as file:
        for point in captured_coordinates:
            x, y, z = point
            file.write(f' {x} {y} {z}\n')
    print(f"Point saved to {filename}")

def main():

    print("☺☺☺☺☺ Author: Chung-Yun Chang ☺☺☺☺☺\n")
    print("Press ' ' to capture points\n")
    print("Press 'y' to save the captured point in txt. file\n")
    print("Press esc or 'q' to close the image window\n")
    print("------------------------------------------------\n")

    pipeline, config = initialize_pipeline()

    # Start streaming
    pipeline.start(config)

    # Getting the depth sensor's depth scale (see rs-align example for explanation)
    profile = pipeline.get_active_profile()
    depth_scale = get_depth_scale(profile)

    # Create an align object
    align = initialize_align()

    # Define the lower and upper HSV range for red color
    lower_red_hue_range = [(160, 160, 60), (120, 100, 120)]
    upper_red_hue_range = [(180, 255, 255), (180, 255, 255)]
    # lower_red_hue_range = [(170,250,180)]
    # upper_red_hue_range = [(180,255,190)]

    # Counter to track the number of captured points
    i = 1

    # List to store captured coordinates
    captured_coordinates = []

    # Create the filename with the timestamp
    filename = create_timestamped_filename()

    # Streaming loop
    try:
        while True:
            # Get frameset of color and depth
            frames = pipeline.wait_for_frames()
            # frames.get_depth_frame() is a 640x360 depth image

            # Align the depth frame to color frame
            aligned_frames = align.process(frames)

            # Get aligned frames
            aligned_depth_frame = aligned_frames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image
            color_frame = aligned_frames.get_color_frame()

            # Validate that both frames are valid
            if not aligned_depth_frame or not color_frame:
                continue
            
            # Converting to numpy
            color_image = np.asanyarray(color_frame.get_data())

            # Convert color image to HSV
            hsv_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)

            # Initialize masks
            lower_mask = np.zeros_like(hsv_image[:, :, 0], dtype=np.uint8)
            upper_mask = np.zeros_like(hsv_image[:, :, 0], dtype=np.uint8)

            # Create masks for lower and upper HSV ranges
            for lower_range, upper_range in zip(lower_red_hue_range, upper_red_hue_range):
                lower_mask_temp = cv2.inRange(hsv_image, np.array(lower_range), np.array(upper_range))
                lower_mask = cv2.bitwise_or(lower_mask, lower_mask_temp)

            # Combine the masks
            mask = cv2.bitwise_or(lower_mask, upper_mask)
            # cv2.imshow('Binary Image', mask)
            # cv2.waitKey(0)
            # Find contours in the mask
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

            for contour in contours:
                if cv2.contourArea(contour) > cir_pixel:
                    radius = int(math.sqrt(cv2.contourArea(contour) / math.pi))
                    M = cv2.moments(contour)
                    if M["m00"] != 0:
                        cX = int(M["m10"] / M["m00"])
                        cY = int(M["m01"] / M["m00"])
                        # Draw a red hollow circle at the center of the red point
                        cv2.circle(color_image, (cX, cY), radius, (0, 0, 255), 2)  # Red hollow circle

                        key = cv2.waitKey(1)
                        if key == ord(' '):
                            captured_coordinates = []
                            depth_value = aligned_depth_frame.get_distance(cX, cY)
                            depth_intrin = aligned_depth_frame.profile.as_video_stream_profile().intrinsics
                            x, y, z = rs.rs2_deproject_pixel_to_point(depth_intrin, [cX, cY], depth_value)
                            print("cX:", cX, " cY:", cY, "\nx:", x, " y:", y," z:", z)
                            captured_coordinates.append((x, y, z))
                            print(f"Point {i} is captured")

                        elif key == ord('y'):
                            save_coordinates_to_file(filename, captured_coordinates)
                            captured_coordinates = []  # Clear the captured coordinates
                            i += 1

            # Display the color image with red circles
            cv2.namedWindow('Red Point Detection', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('Red Point Detection', color_image)

            # Check for keyboard input
            key = cv2.waitKey(1)
            
            # Press esc or 'q' to close the image window
            if key & 0xFF == ord('q') or key == 27:

                # Exit if 'q' is pressed
                cv2.destroyAllWindows()
                break

    finally:
        # Release resources
        pipeline.stop()

if __name__ == "__main__":
    main()