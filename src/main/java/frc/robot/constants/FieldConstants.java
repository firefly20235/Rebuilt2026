package frc.robot.constants;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;

import java.util.HashMap;

public class FieldConstants {
    public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
    public static final HashMap<Integer, Pose3d> TAG_ID_TO_POSE = fieldLayoutToTagIdToPoseMap();


    private static HashMap<Integer, Pose3d> fieldLayoutToTagIdToPoseMap() {
        final HashMap<Integer, Pose3d> tagIdToPose = new HashMap<>();
        for (AprilTag aprilTag : APRIL_TAG_FIELD_LAYOUT.getTags()) {
            tagIdToPose.put(aprilTag.ID, aprilTag.pose);
        }

        return tagIdToPose;
    }

}
