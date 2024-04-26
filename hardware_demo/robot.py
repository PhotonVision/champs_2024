from wpilib import Timer, SmartDashboard, TimedRobot, Servo, LiveWindow
from wpimath.geometry import Transform3d, Pose3d, Translation3d, Rotation3d
from wpimath.units import inchesToMeters, radiansToDegrees
import math
from photonlibpy.photonCamera import PhotonCamera, setVersionCheckEnabled #VisionLEDMode
from wpimath.filter import SlewRateLimiter, Debouncer


# Origin defined as being directly underneath the servo X pivot on the top surface of the demo board
ORIGIN_POSE = Pose3d()

# Pose of camera sensor, in the above reference frame
CAM_MOUNT_POSE = Pose3d(
    Translation3d(
        inchesToMeters(0.25),
        inchesToMeters(3.6),
        inchesToMeters(1.55),
    ), 
    Rotation3d.fromDegrees(
        0.0,-30.0,0.0 
    )
)

#Intersection of the X and Y rotational axes on the gimbal
GIMBAL_ROT_AXIS_MOUNT_POSE = Pose3d(
    Translation3d(
        inchesToMeters(0.0), 
        inchesToMeters(0.0),
        inchesToMeters(2.5),
    ), 
    Rotation3d.fromDegrees(
        0.0,0.0,0.0 
    )
)

# Calculated transforms
gimbalToOrigin = Transform3d(GIMBAL_ROT_AXIS_MOUNT_POSE,ORIGIN_POSE)
originToCam = Transform3d(ORIGIN_POSE, CAM_MOUNT_POSE)

# Transform from the gimbal rotational axes intersection to the pointer end effector
GIMBAL_ROT_AXIS_TO_POINTER = Transform3d(
    Pose3d(),
    Pose3d(
        Translation3d(
            inchesToMeters(0.0), 
            inchesToMeters(0.0),
            inchesToMeters(0.5),
        ), 
        Rotation3d.fromDegrees(
            0.0,0.0,0.0 
        )
    )
)


# Physical/Mechanical limits
MAX_SERVO_SPEED_DEG_PER_SEC = 90.0
gimbal_x_center_angle = 90.0
gimbal_y_center_angle = 90.0
gimbal_x_max_angle = 45.0
gimbal_y_max_angle = 45.0

def _limit(input:float, limit:float)->float:
    if(input > limit):
        return limit
    elif(input < -1.0 * limit):
        return -1.0 * limit
    else:
        return input

class MyRobot(TimedRobot):

    def robotInit(self):
        # Hardware under control
        self.xAxisServo = Servo(0)
        self.yAxisServo = Servo(1)

        # Angle commands for hardware
        self.xAxisAngleCmd = gimbal_x_center_angle
        self.yAxisAngleCmd = gimbal_y_center_angle
        self.rawXTgt = gimbal_x_center_angle
        self.rawYTgt = gimbal_y_center_angle

        # Limit servo motion to be physically plausable
        self.xAxisSlewRate = SlewRateLimiter(MAX_SERVO_SPEED_DEG_PER_SEC)
        self.yAxisSlewRate = SlewRateLimiter(MAX_SERVO_SPEED_DEG_PER_SEC)

        # Debouncer to re-home to center when no target visible
        self.goHomeDbnc = Debouncer(1.0, Debouncer.DebounceType.kRising)


        #Disable livewindow so we can use test for our own purposes
        LiveWindow.disableAllTelemetry()
        LiveWindow.setEnabled(False)

        # Camera for demo
        #setVersionCheckEnabled(False)
        self.cam = PhotonCamera("DEMO")

        # Range Display
        self.distAlongGroundMM = -1.0


    def robotPeriodic(self) -> None:
        SmartDashboard.putNumber("X Angle Cmd", self.xAxisAngleCmd)
        SmartDashboard.putNumber("Y Angle Cmd", self.yAxisAngleCmd)
        SmartDashboard.putNumber("Range (mm)", self.distAlongGroundMM)

        self.xAxisServo.setAngle(self.xAxisAngleCmd)
        self.yAxisServo.setAngle(self.yAxisAngleCmd)


    def teleopPeriodic(self) -> None:
        # Simple - just center the gimbal to test centering
        self.xAxisAngleCmd = gimbal_x_center_angle
        self.yAxisAngleCmd = gimbal_y_center_angle

    def testPeriodic(self) -> None:
        # More complex - run in a large circle to test extents
        self.xAxisAngleCmd = gimbal_x_max_angle * math.sin(2 * math.pi * Timer.getFPGATimestamp() * 0.25) + gimbal_x_center_angle
        self.yAxisAngleCmd = gimbal_y_max_angle * math.cos(2 * math.pi * Timer.getFPGATimestamp() * 0.25) + gimbal_y_center_angle

    def autonomousPeriodic(self) -> None:

        res = self.cam.getLatestResult()

        curCamToTarget = None
        curTrackedID = -1

        for target in res.getTargets():
            # Transform both poses to on-field poses
            tgtID = target.getFiducialId()

            if(tgtID > curTrackedID):
                # Track the highest ID target
                curTrackedID = tgtID
                curCamToTarget = target.getBestCameraToTarget()

        targetVisible = (curTrackedID != -1)
        shouldHome = self.goHomeDbnc.calculate(not targetVisible)



        if(targetVisible):

            # Looking to calculate gimbal to target
            pointerToTarget = GIMBAL_ROT_AXIS_MOUNT_POSE.transformBy(GIMBAL_ROT_AXIS_TO_POINTER).transformBy(gimbalToOrigin).transformBy(originToCam).transformBy(curCamToTarget)

            xAngle = radiansToDegrees(math.atan2(pointerToTarget.y, pointerToTarget.x))
            yAngle = radiansToDegrees(math.atan2(pointerToTarget.z, pointerToTarget.x))

            xAngle = _limit(xAngle, gimbal_x_max_angle)
            yAngle = _limit(yAngle, gimbal_y_max_angle)

            self.rawXTgt = xAngle + gimbal_x_center_angle
            self.rawYTgt = yAngle + gimbal_y_center_angle
            self.distAlongGroundMM = pointerToTarget.x * 1000

        elif(shouldHome):
            self.rawXTgt = gimbal_x_center_angle
            self.rawYTgt = gimbal_y_center_angle
            self.distAlongGroundMM = -1.0

        else:
            # Hold previous position
            pass

        self.xAxisAngleCmd = self.xAxisSlewRate.calculate(self.rawXTgt)
        self.yAxisAngleCmd = self.yAxisSlewRate.calculate(self.rawYTgt)
        



            
            