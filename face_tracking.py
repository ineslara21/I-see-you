import time
import cv2
import PID
import RPi.GPIO as GPIO
from time import sleep
from picamera.array import PiRGBArray
from picamera import PiCamera

# SET INITIAL PARAMETERS -------------------------------------------------------
ok = True
objectSelected=False
frameSize = (320,240)
frameCenter = (frameSize[0]/2,frameSize[1]/2)
# Are we using the Pi Camera?
usingPiCamera = True
camera = PiCamera()
camera.resolution = frameSize
camera.framerate = 30
camera.rotation = 180
rawCapture = PiRGBArray(camera, size=frameSize)

Pan_gpioPIN = 18
Tilt_gpioPIN = 22
Pan_initPosition = 90
Tilt_initPosition = 90
Pan_Position = Pan_initPosition
Tilt_Position = Tilt_initPosition

face_cascade =  cv2.CascadeClassifier('/home/pi/opencv-3.3.0/data/haarcascades/haarcascade_frontalface_alt2.xml')

print("[INFO] Initial parameters set")

# PID'S DEFINITION -------------------------------------------------------------
Pan_pidTarget = 0
Tilt_pidTarget = 0

Pan_Kp = 0.1
Pan_Ki = 0.01
Pan_Kd = 0.001
Tilt_Kp = -0.1
Tilt_Ki = 0.01
Tilt_Kd = 0.001

Pan_pid = PID.PID(Pan_Kp, Pan_Ki, Pan_Kd)
Pan_pid.SetPoint = Pan_pidTarget
Pan_pid.setSampleTime(1)
Tilt_pid = PID.PID(Tilt_Kp, Tilt_Ki, Tilt_Kd)
Tilt_pid.SetPoint = Tilt_pidTarget
Tilt_pid.setSampleTime(1)

print("[INFO] PID Started")

# GPIO DEFINITION --------------------------------------------------------------
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(Pan_gpioPIN, GPIO.OUT)
GPIO.setup(Tilt_gpioPIN, GPIO.OUT)

print("[INFO] GPIO ready")

# CALCULATE ANGLE AND MOVE SERVOS FUNCTION -------------------------------------
def setServoAngle(servo, angle):
    pwm = GPIO.PWM(servo, 50)
    pwm.start(0)
    if angle <0:
        angle = 1
        print ("[ERROR] Too far")
    elif angle > 180:
        angle = 180
        print ("[ERROR] Too far")
    dutyCycle = angle / 18. + 3.
    pwm.ChangeDutyCycle(dutyCycle)
    sleep(0.1)
    pwm.stop()

# CALCULATE PID ERROR FUNCTION -------------------------------------------------
def calculatePIDerror(i,obj_center):
    # The same function for both servos
    # Known: Image frame center and object position in frame (from tracking)
    pid_error = frameCenter[i] - obj_center
    return pid_error


# MOVING SERVOS ----------------------------------------------------------------
print("[INFO] Moving servos to initial position")
setServoAngle(Pan_gpioPIN, Pan_initPosition)
setServoAngle(Tilt_gpioPIN, Tilt_initPosition)
time.sleep(1)

display_window = cv2.namedWindow("image")

for frame in camera.capture_continuous(rawCapture, format="bgr",  use_video_port=True):
    img = frame.array
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    faces = face_cascade.detectMultiScale(gray, 1.1, 5)

    for (x,y,w,h) in faces:
        cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,0),2)
        obj_center = (x+w/2, frameSize[1]-(y+h/2))
        print("[UPDATE] Object center:  x->%d y->%d" %(obj_center[0],obj_center[1]))
        cv2.circle(img,obj_center,2,(255,0,0),-1)
        cv2.circle(img,frameCenter,2,(0,255,0),-1)

        Pan_pidError = calculatePIDerror(0,obj_center[0]) #"pan" = 0
        Tilt_pidError = calculatePIDerror(1,obj_center[1])
        # Update pid signal control value
        Pan_pid.update(Pan_pidError)
        Tilt_pid.update(Tilt_pidError)
        # Calculate value to send to servos: position + pid output
        if (Pan_pid.output < 90) and (Pan_pid.output >-90):
            Pan_Position = Pan_Position - Pan_pid.output
            Tilt_Position = Tilt_Position - Tilt_pid.output
            # Move servos to the calculated position
            setServoAngle(Pan_gpioPIN, Pan_Position)
            setServoAngle(Tilt_gpioPIN, Tilt_Position)
            print("[UPDATE] New Pan Angle: %d" %Pan_Position)
            print("[UPDATE] New Tilt Angle: %d" %Tilt_Position)


    resized = cv2.resize(img, (640,480), interpolation = cv2.INTER_AREA)
    cv2.imshow("image",resized)
    #cv2.imshow("image",img)
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        cv2.imwrite('resultado.jpg',img)
        break

    rawCapture.truncate()
    rawCapture.seek(0)

camera.close()
cv2.destroyAllWindows()
