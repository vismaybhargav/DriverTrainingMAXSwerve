package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.Constants.VisionConstants.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import org.photonvision.PhotonCamera;

import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;

public class VisionIOPhotonVision implements VisionIO {
    protected final PhotonCamera camera;
    protected final Transform3d robotToCamera;

    /**
     * Creates a new VisionIOPhotonVision object.
     *
     */
    public VisionIOPhotonVision(String name, Transform3d robotToCamera) {
        camera = new PhotonCamera(name);
        this.robotToCamera = robotToCamera;
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        Set<Short> tagIds = new HashSet<>();
        List<PoseObservation> poseObservations = new LinkedList<>();

        for(var result : camera.getAllUnreadResults()) {
            if(result.hasTargets()) {
                inputs.latestTargetObservation =
                        new TargetObservation(
                                Rotation2d.fromDegrees(result.getBestTarget().getYaw()),
                                Rotation2d.fromDegrees(result.getBestTarget().getPitch()));
            } else {
                inputs.latestTargetObservation = new TargetObservation(new Rotation2d(), new Rotation2d());
            }

            if(result.multitagResult.isPresent()) {
                var multiTagResult = result.multitagResult.get();

                Transform3d fieldToCamera = multiTagResult.estimatedPose.best;
                Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
                Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

                double totalTagDist = 0.0;
                for(var target : result.getTargets()) {
                    totalTagDist += target.getBestCameraToTarget().getTranslation().getNorm();
                }

                tagIds.addAll(multiTagResult.fiducialIDsUsed);

                poseObservations.add(
                        new PoseObservation(
                                Seconds.of(result.getTimestampSeconds()),
                                robotPose,
                                multiTagResult.estimatedPose.ambiguity,
                                multiTagResult.fiducialIDsUsed.size(),
                                totalTagDist / result.getTargets().size(),
                                PoseObservationType.PHOTONVISION));
            } else if(!result.getTargets().isEmpty()) {
                var target = result.getBestTarget();

                var tagPose = TAG_LAYOUT.getTagPose(target.getFiducialId());
                if(tagPose.isPresent()) {
                    Transform3d fieldToTarget =
                            new Transform3d(tagPose.get().getTranslation(), tagPose.get().getRotation());
                    Transform3d cameraToTarget = target.getBestCameraToTarget();
                    Transform3d fieldToCamera = fieldToTarget.plus(cameraToTarget.inverse());
                    Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
                    Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

                    tagIds.add((short) target.getFiducialId());

                    poseObservations.add(
                            new PoseObservation(
                                    Seconds.of(result.getTimestampSeconds()),
                                    robotPose,
                                    target.getPoseAmbiguity(),
                                    1,
                                    cameraToTarget.getTranslation().getNorm(),
                                    PoseObservationType.PHOTONVISION
                            )
                    );
                }
            }
        }

        inputs.latestPoseObservations = new PoseObservation[poseObservations.size()];
        for (int i = 0; i < poseObservations.size(); i++) {
            inputs.latestPoseObservations[i] = poseObservations.get(i);
        }

        inputs.tagIDs = new int[tagIds.size()];
        int i = 0;
        for (short tagId : tagIds) {
            inputs.tagIDs[i++] = tagId;
        }
    }
}
