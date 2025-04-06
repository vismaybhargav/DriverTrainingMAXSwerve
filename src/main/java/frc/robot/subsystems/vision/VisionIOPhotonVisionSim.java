package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import java.util.function.Supplier;

import static frc.robot.Constants.VisionConstants.*;

public class VisionIOPhotonVisionSim extends VisionIOPhotonVision {
    private static VisionSystemSim visionSystemSim;

    private final Supplier<Pose2d> poseSupplier;
    private final PhotonCameraSim cameraSim;

    /**
     * Creates a new VisionIOPhotonVision object.
     *
     */
    public VisionIOPhotonVisionSim(
            String name, Transform3d robotToCamera, Supplier<Pose2d> poseSupplier
    ) {
        super(name, robotToCamera);
        this.poseSupplier = poseSupplier;

        if(visionSystemSim == null) {
            visionSystemSim = new VisionSystemSim("main");
            visionSystemSim.addAprilTags(TAG_LAYOUT);
        }

        var cameraProps = new SimCameraProperties();
        cameraSim = new PhotonCameraSim(camera, cameraProps, TAG_LAYOUT);
        visionSystemSim.addCamera(cameraSim, robotToCamera);
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        visionSystemSim.update(poseSupplier.get());
        super.updateInputs(inputs);
    }
}
