package frc.robot;

import java.util.List;

import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.math.geometry.Pose2d;

class TagDetection {
    public final int id;
    public final List<TargetCorner> corners;
    public final Pose2d poseGuess;

    public TagDetection(int id, List<TargetCorner> corners, Pose2d guess) {
        this.id = id;
        this.corners = corners;
        this.poseGuess = guess;
    }

    public static final TagDetectionStruct struct = new TagDetectionStruct();
}
