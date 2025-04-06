package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Time;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
    @AutoLog
    public static class VisionIOInputs {
        public TargetObservation latestTargetObservation =
                new TargetObservation(new Rotation2d(), new Rotation2d());
        public PoseObservation[] latestPoseObservations = new PoseObservation[0];
        public int[] tagIDs = new int[0];
    }

    /** Represents the angle to a simple target, not used for pose estimation */
    public static record TargetObservation(Rotation2d tx, Rotation2d ty) {}

    /** Represents a robot pose sample used for pose estimation */
    public static record PoseObservation(
            Time timestamp,
            Pose3d pose,
            double ambiguity,
            int tagCount,
            double avgTagDist,
            PoseObservationType type
    ) {}

    public static enum PoseObservationType {
        PHOTONVISION,
        QUESTNAV
    }

    public default void updateInputs(VisionIOInputs inputs) {}
}
