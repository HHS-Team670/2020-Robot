import cv2
import numpy as np
import glob
import datetime # for telling how long processing takes
import math # for dealing with the translation and rotation vectors
from operator import itemgetter # for sorting arrays
from networktables import NetworkTables # Network tables
import threading
import time
import os

# Original
# lower_green = np.array([58, 50, 0])
# upper_green = np.array([104, 139, 255])
lower_green = np.array([70, 235, 255])
upper_green = np.array([95, 255, 255])
# Network tables info
ERROR = -99999

ROBORIO_IP = "10.6.70.2"
NETWORK_TABLE_NAME = "SmartDashboard"
NETWORK_KEY = "vision-data"
RETURN_NETWORK_KEY = "vision_values"

# Network table (by default returns error codes, but changes in program)
returns = [ERROR, ERROR, ERROR]
cond = threading.Condition()
notified = [False]
enabled_key = "vision-enabled"

# Real world points in millimeters
model_points = np.array([
        (-498.475, 431.8, 0.0),    # upper left
        # (-498.475 + 57.15, 431.8, 0.0),    # upper inner left
        (-250.825, 0.0, 0.0),      # Lower left
        (250.825, 0.0, 0.0),        # Lower right
        # (498.475 - 57.15, 431.8, 0.0),       # Upper inner right
        (498.475, 431.8, 0.0)       # Upper right
])
                                                
# Calibration stats/Camera internals
camera_matrix = np.array([
        [1092.76656, 0, 619.871926],
        [0, 1087.4006, 372.032392],
        [0, 0, 1]], dtype = "double")

dist_coeffs = np.array([.150140071, -1.53360273, -.00222009517, .00272789969])

# The angle that the camera is tilted forward or back at
tilt_angle = 25 # Probably in degrees
# parameters of the camera mount: tilt (up/down), angle inward, and offset from the robot center
# NOTE: the rotation matrix for going "camera coord" -> "robot coord" is computed as R_inward * R_tilt
#   Make sure angles are measured in the right order
CAMERA_TILT = math.radians(tilt_angle)
CAMERA_ANGLE_INWARD = math.radians(0)
CAMERA_OFFSET_X = 0                 # inches left/right from center of rotation
CAMERA_OFFSET_Z = 0                 # inches front/back from C.o.R.
FRONT_CAPTURE_SOURCE = 2          # Video source number for front camera

#Values for contour candidates
ANGLE_THRESHOLD = 8*math.pi/180 # 8 degrees in radians
TARGET_ANGLE = 1.04763098 #radians

#os.system("sudo rm -f /dev/video" + str(FRONT_CAPTURE_SOURCE))
#os.system("sudo ln -s /dev/v4l/by-path/platform-3f980000.usb-usb-0:1.3:1.0-video-index0 /dev/video"+ str(FRONT_CAPTURE_SOURCE))

#cap = cv2.VideoCapture(FRONT_CAPTURE_SOURCE)

#Connection Listener for Network Tables
def connectionListener(connected, info):
        with cond:
                print("connected%s"%connected)
                notified[0] = True
                cond.notify()

def push_network_table(table, return_list):
        '''
        Pushes a tuple to the network table
        prints the table in Debug mode
        '''
        table.putNumberArray(RETURN_NETWORK_KEY, return_list)
        NetworkTables.flush()

# This function takes in the hull of a contour and finds the outer 4 points and returns them
def findOuterPoints(points):
        points = sorted(points, key=itemgetter(0,1))
        corners = []
        
        while (len(points) > 4):
                distances = []
                for i in range(1, len(points) - 1):
                        a = points[i - 1]
                        b = points[i + 1]
                        d = a[1] - b[1]
                        e = b[0] - a[0]
                        f = a[0] * b[1] - b[0] * a [1]
                        p = points[i]
                        perpen_dist = abs(d * p[0] + e * p[1] + f) / math.sqrt(d * d + e * e)
                        distances.append([i, perpen_dist])
                        
                        
                        #A = previous point
                        #B = next point
                        # D= Ay-By
                        # E = Bx-Ax
                        #F = AxBy - BxAy
                        #p = current point
                        #perpen_dist = abs(D*px + E * py + F)/sqrt(D^2 + E^2)
                        #add to new array with i and distance
                #sort by distance and get 2 top ones
                
                distances = sorted(distances, key=itemgetter(1))
                points.remove(points[distances[0][0]])
        #print(points)
        
        for i in range(len(points)):
                corners.append(points[i])
        
        return corners

def test_candidate_contour(candidates):
        '''Determine the true target contour out of potential candidates'''
        max_iterations = 5
        x = 0
        for c in candidates:
                # print(x)
                if x > max_iterations:
                        break
                
                box_points = cv2.convexHull(c)
                points = []

                for i in range(len(box_points)):
                        points.append([box_points[i][0][0], box_points[i][0][1]])

                # print()
                box_points1 = findOuterPoints(points)
                # print(box_points1)
                try:
                        angle1 = math.atan((box_points1[0][1]-box_points1[1][1])/(box_points1[0][0]-box_points1[1][0]))
                        angle2 = math.atan((box_points1[2][1]-box_points1[3][1])/(box_points1[2][0]-box_points1[3][0]))
                except:
                        pass

                if abs(angle1-TARGET_ANGLE) <= ANGLE_THRESHOLD and abs(angle2+TARGET_ANGLE) <= ANGLE_THRESHOLD:
                        print("true")
                        return box_points1

                x+=1 

        return np.array([])

def find_largest_contours(image):
        '''
        Finds the largest contour in the inputted image.
        Returns the minimum area rectangle of that contour.
        If no contours are found, returns -1.
        '''
        # Blurs the image for better contour detection accuracy
        # blur_image = cv2.medianBlur(image, 5)
        # blur_image = cv2.GaussianBlur(blur_image, (5, 5), 0)
        # blur_image = image.copy()
        # Finds ALL the contours of 
        # the image
        # Note: the tree and chain things could probably be optimized better.
        contours, _ = cv2.findContours(image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        try:
                if len(contours) != 0:
                # Find the biggest area contour
                        try:
                                contours.sort(key=cv2.contourArea, reverse=True)
                                return contours
                        except:
                                return np.array([])
                else:
                        return np.array([])
        except:
                return np.array([])

        

# This function gets the rotation and translation vector returned from solvepnp and does math to figure out the actual distance and compensate for the rotation
def compute_output_values(rvec, tvec):
                '''Compute the necessary output distance and angles'''
                x_r_w0 = np.matmul(rot_robot, tvec) + t_robot
                x = x_r_w0[0][0]
                z = x_r_w0[2][0]
                # distance in the horizontal plane between robot center and target
                distance = math.sqrt(x**2 + z**2)
                #print("x: " + str(x/25.4))
                #print("z: " + str(z/25.4))
                # horizontal angle between robot center line and target
                angle1 = math.atan2(x, z)
                rot, _ = cv2.Rodrigues(rvec)
                rot_inv = rot.transpose()
                # location of Robot (0,0,0) in World coordinates
                x_w_r0 = np.matmul(rot_inv, camera_offset_rotated - tvec)
                angle2 = math.atan2(x_w_r0[0][0], x_w_r0[2][0])
                return distance, angle1, angle2

def checkEnabled(table, key, value, isNew):
        timer = datetime.datetime.now()
        cap = cv2.VideoCapture(FRONT_CAPTURE_SOURCE)
        # Upscale the image
        cap.set(3, 1280)
        cap.set(4, 720)

        for x in range(4): #reset buffer
            cap.grab()
        _, frame = cap.read()
        print("starting")
        print()

        size = frame.shape
        # img = cv2.GaussianBlur(frame, (17, 17), 0)
        img = frame
        
        # convert BGR to HSV
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        
        # Default network table values for angles and distances
        returns = [ERROR, ERROR, ERROR]
        
        # get only green colors from image
        color_mask = cv2.inRange(hsv, lower_green, upper_green)
        
        # Bitwise-AND mask and original image
        res = cv2.bitwise_and(img, img, mask=color_mask)
        res = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)
        contour_list = find_largest_contours(res)
        hopefully_hexagon = test_candidate_contour(contour_list)
        print(hopefully_hexagon)

        cv2.imwrite("/tmp/mask" + str(timer) + ".jpg", res)
        cv2.imwrite("/tmp/frame" + str(timer) + ".jpg", frame)

        cv2.imwrite("../srv/static/picture/res.png", res)

        table.putBoolean("vision-frame-updated", True)
        
        if len(hopefully_hexagon) > 0:
                box_points1 = hopefully_hexagon
                print()
                print()
                print(box_points1)

                # cv2.circle(res, (box_points1[0][0],box_points1[0][1]), 1, (120, 120, 120), 7)
                # cv2.circle(res, (box_points1[1][0],box_points1[1][1]), 1, (120, 120, 120), 7)
                # cv2.circle(res, (box_points1[2][0],box_points1[2][1]), 1, (120, 120, 120), 7)
                # cv2.circle(res, (box_points1[3][0],box_points1[3][1]), 1, (120, 120, 120), 7)
                
                if len(box_points1) == 4:
                        image_points = np.array([
                                (box_points1[0][0], box_points1[0][1]),
                                (box_points1[1][0], box_points1[1][1]),
                                (box_points1[2][0], box_points1[2][1]),
                                (box_points1[3][0], box_points1[3][1])
                        ], dtype="double")
                
                        (success, rotation_vector, translation_vector) = cv2.solvePnP(model_points, image_points, camera_matrix, dist_coeffs, flags=cv2.SOLVEPNP_ITERATIVE)
                        
                        # Trying to deal with angles
                        if success:
                                distance, angle1, angle2 = compute_output_values(rotation_vector, translation_vector)
                                print("Calc: " + str(distance / 25.4))
                                print("Angle: " + str(angle2 * 180 / math.pi))
                                # TODO: Check the min and max possible angles before sending to network table
                                returns = [angle1 * 180 / math.pi, angle2 * 180 / math.pi, distance / 25.4]

        print(str((datetime.datetime.now() - timer).microseconds / 1000) + "ms" )

        # cv2.imshow("Test", res)
        # k = cv2.waitKey(5) & 0xFF
        # if k == 27:
        #       return

        # Push network table
        push_network_table(table, returns)
        table.putString(enabled_key, "disabled")
        cap.release()

# Change the physical camera settings
os.system("v4l2-ctl -d /dev/video" +  str(FRONT_CAPTURE_SOURCE)+" -c exposure_auto=1 -c exposure_absolute=20 -c brightness=30 -c white_balance_temperature_auto=0 -c backlight_compensation=10 -c contrast=10 -c saturation=200")
# matrices used to compute coordinates
t_robot = np.array(((CAMERA_OFFSET_X,), (0.0,), (CAMERA_OFFSET_Z,)))
c_a = math.cos(CAMERA_TILT)
s_a = math.sin(CAMERA_TILT)
r_tilt = np.array(((1.0, 0.0, 0.0), (0.0, c_a, -s_a), (0.0, s_a, c_a)))
c_a = math.cos(CAMERA_ANGLE_INWARD)
s_a = math.sin(CAMERA_ANGLE_INWARD)
r_inward = np.array(((c_a, 0.0, -s_a), (0.0, 1.0, 0.0), (s_a, 0.0, c_a)))
rot_robot = np.matmul(r_inward, r_tilt)
camera_offset_rotated = np.matmul(rot_robot.transpose(), -t_robot)

#Initializes connection to RIO
NetworkTables.initialize(server=ROBORIO_IP)

NetworkTables.addConnectionListener(connectionListener, immediateNotify=True)
#Waits until RIO is connected before continuing
with cond:
   if not notified[0]:
       print("waiting")
       cond.wait()

table = NetworkTables.getTable(NETWORK_TABLE_NAME)
table.addEntryListener(checkEnabled)

while True:
        time.sleep(1)

cap.release()
cv2.destroyAllWindows()
