package frc.robot;

import java.util.List;

import org.opencv.core.Point;
import org.photonvision.targeting.TargetCorner;

class TagDetection {
    public final int id;
    public final List<TargetCorner> corners;

    public TagDetection(int id, List<TargetCorner> corners) {
        this.id = id;
        this.corners = corners;
    }

    public static final TagDetectionStruct struct = new TagDetectionStruct();
}
