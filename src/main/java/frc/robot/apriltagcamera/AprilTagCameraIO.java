package frc.robot.apriltagcamera;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Constants;
import frc.robot.Robot;
import org.littletonrobotics.junction.AutoLog;

public class AprilTagCameraIO {
    static AprilTagCameraIO generateIO(AprilTagCameraConstants.AprilTagCameraType aprilTagCameraType, String name, Transform3d robotToCamera) {
        return aprilTagCameraType.createIOFunction.apply(name, robotToCamera);
    }

//    protected void updateInputs(AprilTagCameraInputsAutoLogged inputs) {
//    }

    @AutoLog
    public static class AprilTagCameraInputs {
        public boolean hasTarget = false;
        public Pose3d solvePNPPose = new Pose3d();
        public double latestResultTimestampSeconds = 0;
        public int[] visibleTagIDs = new int[0];
        public double poseAmbiguity = 1;
        public double[] distancesFromTags = new double[0];
    }

}