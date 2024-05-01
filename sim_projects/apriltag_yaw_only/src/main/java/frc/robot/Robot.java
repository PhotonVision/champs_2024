/*
 * MIT License
 *
 * Copyright (c) PhotonVision
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Twist3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSub;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.drivetrain.SwerveDrive;

import static frc.robot.Constants.Vision.kRobotToCam;
import static frc.robot.Constants.Vision.kTagLayout;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.photonvision.targeting.PhotonPipelineResult;

public class Robot extends TimedRobot {
    public static final double PERIOD = 0.01;
    private SwerveDrive drivetrain;
    private Vision vision;

    private XboxController controller;
    // Limit max speed
    private final double kDriveSpeed = 0.6;
    // Rudimentary limiting of drivetrain acceleration
    private SlewRateLimiter forwardLimiter = new SlewRateLimiter(1.0 / 0.6); // 1 / x seconds to 100%
    private SlewRateLimiter strafeLimiter = new SlewRateLimiter(1.0 / 0.6);
    private SlewRateLimiter turnLimiter = new SlewRateLimiter(1.0 / 0.33);

    private Timer autoTimer = new Timer();
    private Random rand = new Random(4512);

    // simple PID controller to aim at the target
    private PIDController aimController = new PIDController(0.02, 0, 0);

    GtsamInterface gtsamInterface = new GtsamInterface();

    private PhotonPipelineResult lastResult = new PhotonPipelineResult();

    public Robot() {
        super(PERIOD);
    }

    @Override
    public void robotInit() {
        drivetrain = new SwerveDrive();
        vision = new Vision();

        controller = new XboxController(0);
    }

    @Override
    public void robotPeriodic() {
        var loopStart = WPIUtilJNI.now();

        drivetrain.periodic();

        // Log values to the dashboard
        drivetrain.log();

        // send updates to gtsam

        Pose3d guess = null;
        List<TagDetection> dets = new ArrayList<>();
        long tagDetTime = 0;

        var results = vision.getLatestResult();
        if (results.getTimestampSeconds() != lastResult.getTimestampSeconds()) {
            lastResult = results;
            for (var result : results.getTargets()) {
                dets.add(
                        new TagDetection(result.getFiducialId(),
                                result.getDetectedCorners()));
            }
            
            // Totally bogus extra latency
            tagDetTime = loopStart - 10000;

            if (results.getMultiTagResult().estimatedPose.isPresent && results.targets.size() >= 1) {
                var pose = new Pose3d().transformBy(results.getMultiTagResult().estimatedPose.best);
                // assume robot is sitting on the floor to better constraint guess
                guess = new Pose3d(pose.toPose2d());
            }
        } else {
            // duplicate, drop it
            // System.out.println("Duplicate");
        }

        gtsamInterface.setCamIntrinsics(vision.getCamIntrinsics());
        gtsamInterface.sendUpdate(loopStart, tagDetTime, dets, drivetrain.getTwist(), guess, kRobotToCam);
    }

    @Override
    public void disabledPeriodic() {
        drivetrain.stop();
    }

    @Override
    public void autonomousInit() {
        autoTimer.restart();
        var pose = new Pose2d(1, 1, new Rotation2d());
        drivetrain.resetPose(pose, true);
        vision.resetSimPose(pose);
    }

    @Override
    public void autonomousPeriodic() {
        // translate diagonally while spinning
        if (autoTimer.get() < 10) {
            drivetrain.drive(0.5, 0.5, 0.5, false);
        } else {
            autoTimer.stop();
            drivetrain.stop();
        }
    }

    /**
     * Estimate how hard to turn to aim at the SPEAKER
     */
    double getTurnPower() {
        // Correct pose estimate with vision measurements
        var visionEst = vision.getLatestResult();

        if (visionEst.hasTargets()) {
            // Look for the blue speaker
            for (var target : visionEst.getTargets()) {
                if (target.getFiducialId() == 7) {
                    // found blue speaker! Aim at it purely based on yaw
                    var anglePower = aimController.calculate(target.getYaw());

                    return MathUtil.clamp(anglePower, -1, 1);
                }
            }
        }

        // no target found, give up
        return 0;
    }

    @Override
    public void teleopPeriodic() {
        // We will use an "arcade drive" scheme to turn joystick values into target
        // robot speeds
        // We want to get joystick values where pushing forward/left is positive
        double forward = -controller.getLeftY() * kDriveSpeed;
        if (Math.abs(forward) < 0.1)
            forward = 0; // deadband small values
        forward = forwardLimiter.calculate(forward); // limit acceleration
        double strafe = -controller.getLeftX() * kDriveSpeed;
        if (Math.abs(strafe) < 0.1)
            strafe = 0;
        strafe = strafeLimiter.calculate(strafe);

        double turn;
        // bound to "Z" on your keyboard
        if (controller.getRawButton(1)) {
            turn = getTurnPower();
        } else {
            turn = -controller.getRightX() * kDriveSpeed;
        }

        turn = turnLimiter.calculate(turn);
        turn = MathUtil.applyDeadband(turn, 0.1);

        // Convert from joystick values to real target speeds
        forward *= Constants.Swerve.kMaxLinearSpeed;
        strafe *= Constants.Swerve.kMaxLinearSpeed;
        turn *= Constants.Swerve.kMaxLinearSpeed;

        // Command drivetrain motors based on target speeds
        drivetrain.drive(forward, strafe, turn, true);
    }

    @Override
    public void simulationPeriodic() {
        // Update drivetrain simulation
        drivetrain.simulationPeriodic();

        // Update camera simulation
        vision.simulationPeriodic(drivetrain.getSimPose());

        var debugField = vision.getSimDebugField();
        debugField.getObject("EstimatedRobot").setPose(drivetrain.getPose());
        debugField.getObject("EstimatedRobotModules").setPoses(drivetrain.getModulePoses());

        SmartDashboard.putNumberArray("/robot/ground_truth_pose", new double[] {
            drivetrain.getPose().getX(),
            drivetrain.getPose().getY(),
            drivetrain.getPose().getRotation().getRadians()
        });

        // Calculate battery voltage sag due to current draw
        RoboRioSim.setVInVoltage(
                BatterySim.calculateDefaultBatteryLoadedVoltage(drivetrain.getCurrentDraw()));
    }
}
