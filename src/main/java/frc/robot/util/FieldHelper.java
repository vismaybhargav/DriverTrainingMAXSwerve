package frc.robot.util;

import static frc.robot.Constants.VisionConstants.TAG_LAYOUT;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public /* singleton */ class FieldHelper {

    /**
     * Reef Side A starts at the one closest to the driver station wall, and then moves counter clockwise around
     */
    public static enum ReefSide {
        A,
        B,
        C,
        D,
        E,
        F,
    }

    public static enum BranchSide {
        LEFT,
        RIGHT
    }

    public static Map<ReefSide, AprilTag> blueReefAprilTags = new HashMap<>();
    public static Map<ReefSide, AprilTag> redReefAprilTags = new HashMap<>();
    
    static {
        blueReefAprilTags.put(ReefSide.A, new AprilTag(18, TAG_LAYOUT.getTagPose(18).orElse(null)));
        blueReefAprilTags.put(ReefSide.B, new AprilTag(17, TAG_LAYOUT.getTagPose(17).orElse(null)));
        blueReefAprilTags.put(ReefSide.C, new AprilTag(22, TAG_LAYOUT.getTagPose(22).orElse(null)));
        blueReefAprilTags.put(ReefSide.D, new AprilTag(21, TAG_LAYOUT.getTagPose(21).orElse(null)));
        blueReefAprilTags.put(ReefSide.E, new AprilTag(20, TAG_LAYOUT.getTagPose(20).orElse(null)));
        blueReefAprilTags.put(ReefSide.F, new AprilTag(19, TAG_LAYOUT.getTagPose(19).orElse(null)));

        redReefAprilTags.put(ReefSide.A, new AprilTag(7, TAG_LAYOUT.getTagPose(7).orElse(null)));
        redReefAprilTags.put(ReefSide.B, new AprilTag(8, TAG_LAYOUT.getTagPose(8).orElse(null)));
        redReefAprilTags.put(ReefSide.C, new AprilTag(9, TAG_LAYOUT.getTagPose(9).orElse(null)));
        redReefAprilTags.put(ReefSide.D, new AprilTag(10, TAG_LAYOUT.getTagPose(10).orElse(null)));
        redReefAprilTags.put(ReefSide.E, new AprilTag(11, TAG_LAYOUT.getTagPose(11).orElse(null)));
        redReefAprilTags.put(ReefSide.F, new AprilTag(6, TAG_LAYOUT.getTagPose(6).orElse(null)));
    }

    private FieldHelper() {
        // Prevent instantiation
    }

    public static Pose2d getAlignedDesiredPoseForReef(ReefSide reefSide, BranchSide branchSide) {
        Map<ReefSide, AprilTag> mapToUse;

        if(DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
            mapToUse = redReefAprilTags;
        } else if(DriverStation.getAlliance().isPresent()) {
            mapToUse = blueReefAprilTags;
        } else {
            throw new IllegalStateException("You are not red or blue alliance!");
        }

        Pose2d atPose = mapToUse.get(reefSide).pose.toPose2d();

        atPose.tran
    }
}
