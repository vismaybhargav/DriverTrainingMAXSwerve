package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;

public class FieldHelper {
    public enum ReefSide {
        A,
        B,
        C,
        D,
        E,
        F,
    }

    public enum BranchSide {
        LEFT,
        RIGHT
    }

    private FieldHelper() {
        // Prevent instantiation
    }

    public static Pose2d getAlignedDesiredPoseForReef(ReefSide reefSide, BranchSide branchSide) {
        return new Pose2d();
    }
}
