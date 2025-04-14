package frc.robot.util;

import static edu.wpi.first.units.Units.Meters;
import static frc.robot.Constants.VisionConstants.TAG_LAYOUT;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;

public /* singleton */ class FieldHelper {

    /**
     * Reef Side A starts at the one closest to the driver station wall, and then moves counter clockwise around
     */
    public static enum ReefSide {
        A, B, C, D, E, F
    }

    public static enum BranchSide {
        LEFT, RIGHT
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
        Map<ReefSide, AprilTag> mapToUse = DriverStation
            .getAlliance()
            .orElse(Alliance.Blue) == Alliance.Blue 
                ? blueReefAprilTags : redReefAprilTags;
        
        Pose2d atPose = mapToUse.get(reefSide).pose.toPose2d();

        Transform2d offsetTransform = new Transform2d(
            DriveConstants.robotWidth.in(Meters) / 2, // Back to Front (Don't change this one)
            branchSide == BranchSide.LEFT ? 
                AutoConstants.REEF_Y_LEFT_OFFSET.in(Meters) : 
                AutoConstants.REEF_Y_RIGHT_OFFSET.in(Meters), // Side to Side
            Rotation2d.k180deg
        );

        return atPose.transformBy(offsetTransform);
    }
}
