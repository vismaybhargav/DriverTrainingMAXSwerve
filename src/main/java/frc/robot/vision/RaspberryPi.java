package frc.robot.vision;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants.VisionConstants;

import java.util.ArrayList;
import java.util.Arrays;

/**
 * The RaspberryPi class is used to interface with the Raspberry Pi on the robot.
 */
public class RaspberryPi {
    private NetworkTable table;
    private final DoubleArraySubscriber tagSubscriber;

    /**
     * Default constructor for the RaspberryPi class.
     */
    public RaspberryPi() {
        table = NetworkTableInstance.getDefault().getTable("datatable");
        tagSubscriber = table.getDoubleArrayTopic("april_tag_data").subscribe(new double[] {});
    }

    /**
     * Prints the raw data for the april tags on the rpi.
     */
    public void printRawData() {
        double[] rawData = tagSubscriber.get();
        System.out.println(Arrays.toString(rawData));
    }

    /**
     * Gets the data from the Raspberry Pi.
     *
     * @return  ArrayList<AprilTag>
     *          The data from the Raspberry Pi
     */
    public ArrayList<AprilTag> getAprilTags() {
        ArrayList<AprilTag> atList = new ArrayList<>();
        double[] rawData = tagSubscriber.get();

        if (rawData.length == 0) {
            return atList;
        }

        for (
                int i = 0;
                i < rawData.length;
                i += VisionConstants.AT_ARR_INC
        ) {
            atList.add(
                    new AprilTag(
                            (int) rawData[i],
                            "Reef Camera",
                            new Translation3d(
                                    rawData[i + VisionConstants.AT_ARR_CAMERA_OFFSET],
                                    rawData[i + VisionConstants.AT_ARR_CAMERA_OFFSET + 1],
                                    rawData[i + VisionConstants.AT_ARR_CAMERA_OFFSET + 2]
                            ),
                            new Translation3d(
                                    rawData[i + VisionConstants.AT_ARR_TRANSLATION_OFFSET],
                                    rawData[i + VisionConstants.AT_ARR_TRANSLATION_OFFSET + 1],
                                    rawData[i + VisionConstants.AT_ARR_TRANSLATION_OFFSET + 2]
                            ),
                            new Rotation3d(
                                    rawData[i + VisionConstants.AT_ARR_ROTATION_OFFSET],
                                    rawData[i + VisionConstants.AT_ARR_ROTATION_OFFSET + 1],
                                    rawData[i + VisionConstants.AT_ARR_ROTATION_OFFSET + 2]
                            )
                    )
            );
        }

        return atList;
    }

    /**
     * Gets an April Tag from the list given a certain tag.
     * @param id id of the april tag
     * @return the april tag matching the id
     */
    public AprilTag getAprilTagWithID(int id) {
        return getAprilTags()
                .stream()
                .filter(tag -> tag.getTagID() == id)
                .findFirst()
                .orElse(null);
    }
}
