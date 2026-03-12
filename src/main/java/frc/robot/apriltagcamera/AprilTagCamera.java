package frc.robot.apriltagcamera;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import org.littletonrobotics.junction.Logger;

public class AprilTagCamera {
    private final String name;
    private final Transform3d cameraToRobot;
    private final AprilTagCameraIO aprilTagCameraIO;
//    private final AprilTagCameraInputsAutoLogged inputs = new AprilTagCameraInputsAutoLogged();
    private Pose2d estimatedRobotPose = new Pose2d();

    public AprilTagCamera(AprilTagCameraConstants.AprilTagCameraType aprilTagCameraType, String name, Transform3d cameraToRobot) {
        this.name = name;
        this.cameraToRobot = cameraToRobot;
        this.aprilTagCameraIO = AprilTagCameraIO.generateIO(aprilTagCameraType, name, cameraToRobot.inverse());

    }

    public void updatePeriodically() {
//        aprilTagCameraIO.updateInputs(inputs);
        estimatedRobotPose = calculateRobotPose();

//        Logger.processInputs("AprilTagCameras/" + name, inputs);//Logs all camera inputs (pose, ambiguity, etc.)
    }

    public Pose2d getEstimatedRobotPose() {
        return estimatedRobotPose;
    }

    public double getLatestResultTimestamp() {
        return 0;
//        return inputs.latestResultTimestampSeconds;
    }

    public boolean hasValidTarget() {
        return false;
//        return inputs.hasTarget && inputs.poseAmbiguity < AprilTagCameraConstants.MAX_AMBIGUITY;
    }

    public Matrix<N3, N1> calculateStandardDeviations() {
        final double averageDistanceFromTags = calculateAverageDistance();
//        final double translationStandardDeviation = calculateStandardDeviations(AprilTagCameraConstants.TRANSLATION_STANDARD_DEVIATION_EXPONENT, averageDistanceFromTags, inputs.visibleTagIDs.length);
//        final double rotationStandardDeviation = calculateStandardDeviations(AprilTagCameraConstants.ROTATION_STANDARD_DEVIATION_EXPONENT, averageDistanceFromTags, inputs.visibleTagIDs.length);

//        return VecBuilder.fill(
//                translationStandardDeviation,
//                translationStandardDeviation,
//                rotationStandardDeviation
//        );
        return null;
    }

    private double calculateAverageDistance() {
        double totalDistanceFromTags = 0;

//        for (double distanceFromTag : inputs.distancesFromTags)
//            totalDistanceFromTags += distanceFromTag;
//
//        return totalDistanceFromTags / inputs.distancesFromTags.length;
        return 0;
    }


    private Pose2d calculateRobotPose() {
//        return inputs.solvePNPPose.transformBy(cameraToRobot).toPose2d();
        return null;
    }

    private double calculateStandardDeviations(double exponent, double distance, int numberOfVisibleTags) {
        return exponent * (distance * distance) / numberOfVisibleTags;
    }
}