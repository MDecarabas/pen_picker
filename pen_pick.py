#Let's see a pen
from interbotix_xs_modules.arm import InterbotixManipulatorXS
# First import the library
import pyrealsense2 as rs
# Import Numpy for easy array manipulation
import numpy as np
# Import OpenCV for easy image rendering
import cv2
lower = [115,100,20]
upper = [155,255,255]

lower = np.array(lower, dtype = "uint8")
upper = np.array(upper, dtype = "uint8")
kernel = np.ones((5,5),np.float32)/25

# Create a pipeline
pipeline = rs.pipeline()
pc = rs.pointcloud()

# Create a config and configure the pipeline to stream
#  different resolutions of color and depth streams
config = rs.config()

# Get device product line for setting a supporting resolution
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
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

config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

if device_product_line == 'L500':
    config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
else:
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
cfg = pipeline.start(config)

# Getting the depth sensor's depth scale (see rs-align example for explanation)
depth_sensor = cfg.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()
print("Depth Scale is: " , depth_scale)

# We will be removing the background of objects more than
#  clipping_distance_in_meters meters away
clipping_distance_in_meters = 1 #1 meter
clipping_distance = clipping_distance_in_meters / depth_scale

# Create an align object
# rs.align allows us to perform alignment of depth frames to others frames
# The "align_to" is the stream type to which we plan to align depth frames.
align_to = rs.stream.color
align = rs.align(align_to)


# The robot object is what you use to control the robot
robot = InterbotixManipulatorXS("px100", "arm", "gripper")
mode = 'h'
# Let the user select the position



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

        depth_image = np.asanyarray(aligned_depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # Remove background - Set pixels further than clipping_distance to grey
        # mask_color = 100
        # depth_image_3d = np.dstack((depth_image,depth_image,depth_image)) #depth image is 1 channel, color is 3 channels
        # bg_removed = np.where((depth_image_3d > clipping_distance) | (depth_image_3d <= 0), mask_color, color_image)
        hsv_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv_image,lower,upper)


        output = cv2.bitwise_and(hsv_image, hsv_image, mask = mask)
        
        img_erosion = cv2.erode(mask, kernel, iterations=2)
        img_dilation = cv2.dilate(img_erosion, kernel, iterations=3)
        edged = cv2.Canny(img_dilation,0, 128)
        contours, hierarchy = cv2.findContours(edged, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        # #cnt = contours[4]
        cv2.drawContours(color_image, contours, -1, (0,255,0), 10)
        
        if len(contours) > 0:
            M = cv2.moments(contours[0])
            cx = int(M['m10']/M['m00']) + 1
            cy = int(M['m01']/M['m00']) + 1
            # stuff = np.array([cx,cy])
            #print(cx, cy)
            # (x,y), radius = cv2.minEnclosingCircle(stuff)
            # center = (int(x),int(y))
            #radius = int(radius)
            cv2.circle(color_image,(int(cx),int(cy)), 1,(0,0,255),2)
            profile = cfg.get_stream(rs.stream.color)
            intrinsics = profile.as_video_stream_profile().get_intrinsics()

            data = depth_image[cy][cx]*depth_scale

            penPos = rs.rs2_deproject_pixel_to_point(intrinsics, [int(cx), int(cy)], data) 
            print(penPos)

            # while mode != 'q':
            #     mode=input("[h]ome, [s]leep, [q]uit ")
            #     if mode == "h":
            #         robot.arm.go_to_home_pose()
            #         #joint_positions = [0, 0, 0, 1.57]
            #         robot.gripper.close()
            #         #robot.gripper.open()
            #         # robot.arm.go_to_home_pose()
            #         # robot.arm.set_joint_positions(joint_positions)

            #         robot.arm.set_ee_cartesian_trajectory(z=-0.1)
            #         robot.arm.set_ee_cartesian_trajectory(x=-0)
            #         robot.arm.go_to_home_pose()
            #     elif mode == "s":
            #         robot.arm.go_to_sleep_pose()


        # # Render images:
        # #   depth align to color on left
        # #   depth on right
        #depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
        #images = np.hstack((mask)) 
        #depth_colormap))


        
        # profile = cfg.get_stream(rs.stream.color)
        # points = pc.calculate(depth_image)
        # pc.map_to(bg_removed)

        cv2.namedWindow('Final Image', cv2.WINDOW_NORMAL)
        cv2.imshow('Final Image', color_image)
        key = cv2.waitKey(1)

        # Press esc or 'q' to close the image window
        if key & 0xFF == ord('q') or key == 27:
            cv2.destroyAllWindows()
            break
finally:
    pipeline.stop()
