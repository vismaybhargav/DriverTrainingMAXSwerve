package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

import static frc.robot.Constants.VisionConstants.*;

import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

public class Vision extends SubsystemBase {
    private final VisionConsumer visionConsumer;
    private final VisionIO[] io;
    private final VisionIOInputsAutoLogged[] inputs;

    public Vision(VisionConsumer visionConsumer, VisionIO... io) {
        this.visionConsumer = visionConsumer;
        this.io = io;

        this.inputs = new VisionIOInputsAutoLogged[io.length];
        for (int i = 0; i < io.length; i++) {
            inputs[i] = new VisionIOInputsAutoLogged();
        }
    }

    /**
     * Returns the yaw angle to the best target, which can be used for simple targeting.
     *
     * @return The yaw angle to the best target.
     */
    public Rotation2d getTargetYaw(int cameraIdx) {
        return inputs[cameraIdx].latestTargetObservation.tx();
    }

    @Override
    public void periodic() {
        for(int i = 0; i < io.length; i++) {
            io[i].updateInputs(inputs[i]);
            Logger.processInputs("Vision/Camera" + i, inputs[i]);
        }

        List<Pose3d> allTagPoses = new LinkedList<>();
        List<Pose3d> allRobotPoses = new LinkedList<>();
        List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
        List<Pose3d> allRobotPosesRejected = new LinkedList<>();

        for (int camIdx = 0; camIdx < io.length; camIdx++) {
            // Initialize logging values
            List<Pose3d> tagPoses = new LinkedList<>();
            List<Pose3d> robotPoses = new LinkedList<>();
            List<Pose3d> robotPosesAccepted = new LinkedList<>();
            List<Pose3d> robotPosesRejected = new LinkedList<>();

            Arrays.stream(inputs[camIdx].tagIDs)
                    .mapToObj(tagID -> TAG_LAYOUT.getTagPose(tagID))
                    .forEach(optPose -> optPose.ifPresent(tagPoses::add));

            for(VisionIO.PoseObservation poseObservation : inputs[camIdx].latestPoseObservations) {
                boolean rejectPose =
                    poseObservation.tagCount() == 0
                        || (poseObservation.tagCount() == 1
                            && poseObservation.ambiguity() > MAX_AMBIGUITY)
                        || Math.abs(poseObservation.pose().getZ()) > MAX_Z_ERROR
                        || poseObservation.pose().getX() < 0.0
                        || poseObservation.pose().getY() < 0.0
                        || poseObservation.pose().getX() > TAG_LAYOUT.getFieldLength()
                        || poseObservation.pose().getY() > TAG_LAYOUT.getFieldWidth();

                robotPoses.add(poseObservation.pose());
                if (rejectPose) {
                    robotPosesRejected.add(poseObservation.pose());
                } else {
                    robotPosesAccepted.add(poseObservation.pose());
                }

                if(rejectPose) {
                    continue;
                }

                double stdDevFactor = Math.pow(poseObservation.avgTagDist(), 2.0) / poseObservation.tagCount();
                double linearStdDev  = LINEAR_STD_DEV_BASELINE * stdDevFactor;
                double angularStdDev = ANGULAR_STD_DEV_BASELINE * stdDevFactor;

                if(camIdx < CAMERA_STD_DEV_FACTORS.length) {
                    linearStdDev *= CAMERA_STD_DEV_FACTORS[camIdx];
                    angularStdDev *= CAMERA_STD_DEV_FACTORS[camIdx];
                }

                visionConsumer.accept(
                        poseObservation.pose().toPose2d(),
                        poseObservation.timestampSeconds(),
                        VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev)
                );
                Logger.recordOutput(
                        "Vision/Camera" + camIdx + "/TagPoses",
                        tagPoses.toArray(new Pose3d[0]));
                Logger.recordOutput(
                        "Vision/Camera" + camIdx + "/RobotPoses",
                        robotPoses.toArray(new Pose3d[0]));
                Logger.recordOutput(
                        "Vision/Camera" + camIdx + "/RobotPosesAccepted",
                        robotPosesAccepted.toArray(new Pose3d[0]));
                Logger.recordOutput(
                        "Vision/Camera" + camIdx + "/RobotPosesRejected",
                        robotPosesRejected.toArray(new Pose3d[0]));
                allTagPoses.addAll(tagPoses);
                allRobotPoses.addAll(robotPoses);
                allRobotPosesAccepted.addAll(robotPosesAccepted);
                allRobotPosesRejected.addAll(robotPosesRejected);
            }
        }

        // Log summary data
        Logger.recordOutput(
                "Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[0]));
        Logger.recordOutput(
                "Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[0]));
        Logger.recordOutput(
                "Vision/Summary/RobotPosesAccepted",
                allRobotPosesAccepted.toArray(new Pose3d[0]));
        Logger.recordOutput(
                "Vision/Summary/RobotPosesRejected",
                allRobotPosesRejected.toArray(new Pose3d[0]));
    }

    @FunctionalInterface
    public static interface VisionConsumer {
        void accept(Pose2d pose2d, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs);
    }
}
