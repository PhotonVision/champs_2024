import random
import wpilib
from wpilib import TimedRobot, Field2d, SmartDashboard
from wpimath.geometry import Pose2d, Pose3d, Translation2d, Rotation2d
from wpimath.units import inchesToMeters
from wpimath.estimator import SwerveDrive4PoseEstimator
from wpimath.kinematics import SwerveDrive4Kinematics
from wpimath.kinematics import SwerveModulePosition
from wpimath.kinematics import ChassisSpeeds
from wpilib import Timer
from ntcore import NetworkTableInstance

# Wheel base half width: Distance from the center of the frame rail
# out to the center of the "contact patch" where the wheel meets the ground
WHEEL_BASE_HALF_WIDTH_M = inchesToMeters(23.75 / 2.0)
WHEEL_BASE_HALF_LENGTH_M = inchesToMeters(23.75 / 2.0)

# Array of translations from robot's origin (center bottom, on floor) to the module's contact patch with the ground
robotToModuleTranslations = []
robotToModuleTranslations.append(
    Translation2d(WHEEL_BASE_HALF_WIDTH_M, WHEEL_BASE_HALF_LENGTH_M)
)
robotToModuleTranslations.append(
    Translation2d(WHEEL_BASE_HALF_WIDTH_M, -WHEEL_BASE_HALF_LENGTH_M)
)
robotToModuleTranslations.append(
    Translation2d(-WHEEL_BASE_HALF_WIDTH_M, WHEEL_BASE_HALF_LENGTH_M)
)
robotToModuleTranslations.append(
    Translation2d(-WHEEL_BASE_HALF_WIDTH_M, -WHEEL_BASE_HALF_LENGTH_M)
)

# WPILib Kinematics object
kinematics = SwerveDrive4Kinematics(
    robotToModuleTranslations[0],
    robotToModuleTranslations[1],
    robotToModuleTranslations[2],
    robotToModuleTranslations[3],
)

initialModulePositions = [
    SwerveModulePosition(),
    SwerveModulePosition(),
    SwerveModulePosition(),
    SwerveModulePosition(),
]

initialPose = Pose2d(0.0, 4.0, 0.0)


class MyRobot(TimedRobot):
    def robotInit(self):

        self.lowStdDevField = Field2d()
        SmartDashboard.putData("DT Pose Low Std Dev", self.lowStdDevField)
        self.highStdDevField = Field2d()
        SmartDashboard.putData("DT Pose High Std Dev", self.highStdDevField)


        self.estPoseLowStdDev = initialPose
        self.estPoseHighStdDev = initialPose

        self.visionPoses = []

        self.poseEstLowStdDev = SwerveDrive4PoseEstimator(
            kinematics, initialPose.rotation(), initialModulePositions, self.estPoseLowStdDev
        )

        self.poseEstHighStdDev = SwerveDrive4PoseEstimator(
            kinematics,initialPose.rotation(), initialModulePositions, initialPose
        )

        self.poseEstHighStdDev.setVisionMeasurementStdDevs([5.0,5.0,1.0])


        self.curModulePos = initialModulePositions

        self.distTraveled = 0.0
        
        self.fwdVel = 1.0
    
    def addVisionObservations(self, observations:list[Pose2d]):
        if(len(observations) > 0):
            for obs in observations:
                self.visionPoses.append(obs)

    def clearVisionObservations(self):
        self.visionPoses = []

    def robotPeriodic(self):
        for pose in self.visionPoses:
            self.poseEstLowStdDev.addVisionMeasurement(pose, Timer.getFPGATimestamp()-0.1)
            self.poseEstHighStdDev.addVisionMeasurement(pose, Timer.getFPGATimestamp()-0.1)

        self.lowStdDevField.getObject("visionObservations").setPoses(self.visionPoses)
        self.highStdDevField.getObject("visionObservations").setPoses(self.visionPoses)



        self.curModStates = tuple( x for x in [
                                   SwerveModulePosition(angle=Rotation2d.fromDegrees(0),distance=self.distTraveled),
                                   SwerveModulePosition(angle=Rotation2d.fromDegrees(0),distance=self.distTraveled),
                                   SwerveModulePosition(angle=Rotation2d.fromDegrees(0),distance=self.distTraveled),
                                   SwerveModulePosition(angle=Rotation2d.fromDegrees(0),distance=self.distTraveled),
                                    ]
                                  )
        self.poseEstHighStdDev.update(initialPose.rotation(), self.curModStates )
        self.poseEstLowStdDev.update(initialPose.rotation(), self.curModStates )

        self.estPoseLowStdDev = self.poseEstLowStdDev.getEstimatedPosition()
        self.estPoseHighStdDev = self.poseEstHighStdDev.getEstimatedPosition()

        self.lowStdDevField.setRobotPose(self.estPoseLowStdDev)
        self.highStdDevField.setRobotPose(self.estPoseHighStdDev)

    def disabledInit(self):
        self.clearVisionObservations()
        self.poseEstHighStdDev.resetPosition(initialPose.rotation(), initialModulePositions, initialPose)
        self.poseEstLowStdDev.resetPosition(initialPose.rotation(), initialModulePositions, initialPose)
        self.distTraveled = 0.0

    def teleopInit(self):
        self.startTime = Timer.getFPGATimestamp()

    def teleopPeriodic(self):
        self.clearVisionObservations()

        dur = Timer.getFPGATimestamp() - self.startTime

        if(dur < 8.0):
            vel = self.fwdVel
        else:
            vel = 0.0

        self.distTraveled += vel * 0.02


        if(dur> 2.0):
            visionPoseEst = Pose2d(
                Translation2d(
                    self.distTraveled + random.uniform(-0.05, 0.05),
                    initialPose.translation().Y() + 2.0 + random.uniform(-0.05, 0.05),
                ),
                initialPose.rotation()
            )
            self.addVisionObservations([visionPoseEst])
