package frc.robot.subsystems.vision;

import static frc.robot.Constants.VisionConstants.TAG_LAYOUT;

import java.util.function.Supplier;

import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;

public class VisionIOPhotonPoseEstimatorSim extends VisionIOPhotonPoseEstimator {
    private static VisionSystemSim visionSystemSim;
    private Supplier<Pose2d> poseSupplier;

    public VisionIOPhotonPoseEstimatorSim(String name, Transform3d robotToCamera, Supplier<Pose2d> poseSupplier) {
        super(name, robotToCamera);
        this.poseSupplier = poseSupplier;

        if (visionSystemSim == null) {
            visionSystemSim = new VisionSystemSim("main");
            visionSystemSim.addAprilTags(TAG_LAYOUT);
        }

        var cameraProps = new SimCameraProperties();
        var cameraSim = new PhotonCameraSim(camera, cameraProps, TAG_LAYOUT);
        visionSystemSim.addCamera(cameraSim, robotToCamera);
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        visionSystemSim.update(poseSupplier.get());
        super.updateInputs(inputs);
    }
}
