package frc.robot;

import java.util.List;
import java.util.Optional;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;

public class GtsamInterface {
    StructArrayPublisher<TagDetection> tagPub;
    StructPublisher<Twist3d> odomPub;
    StructPublisher<Pose3d> guessPub;
    DoubleArrayPublisher camIntrinsicsPublisher;
    StructPublisher<Transform3d> robotTcamPub;

    public GtsamInterface() {
        tagPub = NetworkTableInstance.getDefault()
                .getStructArrayTopic("/gtsam_meme/sim_camera1/input/tags", TagDetection.struct)
                .publish(PubSubOption.sendAll(true), PubSubOption.keepDuplicates(true));
        odomPub = NetworkTableInstance.getDefault()
                .getStructTopic("/gtsam_meme/input/odom_twist", Twist3d.struct)
                .publish(PubSubOption.sendAll(true), PubSubOption.keepDuplicates(true));
        guessPub = NetworkTableInstance.getDefault()
                .getStructTopic("/gtsam_meme/input/pose_initial_guess", Pose3d.struct)
                .publish(PubSubOption.sendAll(true), PubSubOption.keepDuplicates(true));
        robotTcamPub = NetworkTableInstance.getDefault()
                .getStructTopic("/gtsam_meme/sim_camera1/input/robotTcam", Transform3d.struct)
                .publish(PubSubOption.sendAll(false), PubSubOption.keepDuplicates(false));
        camIntrinsicsPublisher = NetworkTableInstance.getDefault()
                .getDoubleArrayTopic("/gtsam_meme/sim_camera1/input/cam_intrinsics")
                .publish(PubSubOption.sendAll(false), PubSubOption.keepDuplicates(false));
    }

    public void setCamIntrinsics(Optional<Matrix<N3, N3>> intrinsics) {
        if (intrinsics.isEmpty()) {
            return;
        }
        camIntrinsicsPublisher.set(new double[] {
            intrinsics.get().get(0, 0),
            intrinsics.get().get(1, 1),
            intrinsics.get().get(0, 2),
            intrinsics.get().get(1, 2),
        });
    }

    public void sendUpdate(long odomTime, long tagDetTime, List<TagDetection> dets, Twist3d odom, Pose3d guess, Transform3d robotTcam) {

        tagPub.set(dets.toArray(new TagDetection[0]), tagDetTime);

        odomPub.set(odom, odomTime);
        if (guess != null) {
            guessPub.set(guess, odomTime);
        }
        robotTcamPub.set(robotTcam);

        NetworkTableInstance.getDefault().flush();
    }
}
