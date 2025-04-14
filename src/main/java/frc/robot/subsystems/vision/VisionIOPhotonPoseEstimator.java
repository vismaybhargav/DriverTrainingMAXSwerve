package frc.robot.subsystems.vision;

import static frc.robot.Constants.VisionConstants.TAG_LAYOUT;

import java.util.LinkedList;
import java.util.List;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;

public class VisionIOPhotonPoseEstimator extends VisionIOPhotonVision {
    private PhotonPoseEstimator poseEstimator;

    public VisionIOPhotonPoseEstimator(String name, Transform3d robotToCamera) {
        super(name, robotToCamera);
        poseEstimator = new PhotonPoseEstimator(TAG_LAYOUT, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCamera);
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        var results = camera.getAllUnreadResults();
        List<PoseObservation> poseObservations = new LinkedList<>();
        List<TargetObservation> targetObservations = new LinkedList<>();

        for(PhotonPipelineResult result : results) {
            var targets = result.getTargets();
            
            for(PhotonTrackedTarget target : targets) {
                if (target != null) {
                    targetObservations.add(
                            new TargetObservation(
                                    Rotation2d.fromDegrees(target.getYaw()),
                                    Rotation2d.fromDegrees(target.getPitch())));
                }
            }
            

            var pose = poseEstimator.update(result, camera.getCameraMatrix(), camera.getDistCoeffs());

            if (pose.isPresent()) {
                EstimatedRobotPose estPose = pose.get();
                
                var totalTagDist = 0.0;
                for(PhotonTrackedTarget targetUsed : estPose.targetsUsed) {
                    totalTagDist += targetUsed.getBestCameraToTarget().getTranslation().getNorm();
                }
                var averageTagDist = totalTagDist / estPose.targetsUsed.size();

                poseObservations.add(new PoseObservation(
                        estPose.timestampSeconds,
                        estPose.estimatedPose,
                        0, //TODO: How do we get the ambiguity from this?
                        estPose.targetsUsed.size(),
                        averageTagDist,
                        PoseObservationType.PHOTONVISION
                ));
            }
        }

        inputs.latestPoseObservations = poseObservations.toArray(new PoseObservation[0]);
        inputs.latestTargetObservation = targetObservations.isEmpty() ? null : targetObservations.get(0);
    }
}
