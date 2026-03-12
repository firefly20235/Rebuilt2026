package frc.robot.apriltagcamera;

import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.apriltagcamera.io.AprilTagLimelightCameraIO;


import java.util.function.BiFunction;

public class AprilTagCameraConstants {
    static final double MAX_AMBIGUITY=0.4;
    static final double TRANSLATION_STANDARD_DEVIATION_EXPONENT= 0.0002;
    static final double ROTATION_STANDARD_DEVIATION_EXPONENT= 0.0002;
    public enum AprilTagCameraType {
        LIMELIGHT(AprilTagLimelightCameraIO::new);

        final BiFunction<String, Transform3d, AprilTagCameraIO> createIOFunction;

        AprilTagCameraType(BiFunction<String, Transform3d, AprilTagCameraIO> createIOFunction) {
            this.createIOFunction = createIOFunction;
        }
    }
}