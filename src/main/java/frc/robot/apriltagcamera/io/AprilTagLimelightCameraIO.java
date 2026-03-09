package frc.robot.apriltagcamera.io;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.apriltagcamera.AprilTagCameraIO;
//import frc.robot.apriltagcamera.AprilTagCameraInputsAutoLogged;
import frc.robot.apriltagcamera.LimelightHelpers;
import frc.robot.constants.FieldConstants;

public class AprilTagLimelightCameraIO extends AprilTagCameraIO {
    private final String limelightName;

    public AprilTagLimelightCameraIO(String limelight, Transform3d robotToCamera) {
        this.limelightName = limelight;
    }

//    protected void updateInputs(AprilTagCameraInputsAutoLogged inputs) {
//        final LimelightHelpers.LimelightResults results = LimelightHelpers.getLatestResults(limelightName);
//
//        inputs.hasTarget = results != null && results.valid;
//        if (inputs.hasTarget) {
//            updateHasResultInputs(inputs, results);
//            return;
//        }
//        updateNoResultInputs(inputs);
//
//    }

//    private void updateHasResultInputs(AprilTagCameraInputsAutoLogged inputs, LimelightHelpers.LimelightResults results) {
//
//        inputs.solvePNPPose = results.getBotPose3d_wpiBlue();
//        inputs.visibleTagIDs = getVisibleTagIDs(results);
//        inputs.distancesFromTags = calculateDistancesFromTags(results, inputs.solvePNPPose, inputs.visibleTagIDs);
//        inputs.latestResultTimestampSeconds = results.timestamp_RIOFPGA_capture;
//
//    }

    private double[] calculateDistancesFromTags(LimelightHelpers.LimelightResults results, Pose3d robotPose, int[] visibleTagIDs) {
        double[] distancesFromTags = new double[visibleTagIDs.length];
        for (int i = 0; i < distancesFromTags.length; i++) {
            int currentTagID = visibleTagIDs[i];
            Pose3d tagPose = FieldConstants.TAG_ID_TO_POSE.get(currentTagID);
            distancesFromTags[i] = robotPose.getTranslation().getDistance(tagPose.getTranslation());//TODO: ask ai to calc distance from ll angle
        }
        return distancesFromTags;
    }

    private int[] getVisibleTagIDs(LimelightHelpers.LimelightResults results) {

        int[] tagIDs = new int[results.targets_Fiducials.length];

        for (int i = 0; i < tagIDs.length; i++) {
            tagIDs[i] = (int) results.targets_Fiducials[i].fiducialID;
        }

        return tagIDs;
    }

//    private void updateNoResultInputs(AprilTagCameraInputsAutoLogged inputs) {
//        inputs.solvePNPPose = new Pose3d();
//        inputs.visibleTagIDs = new int[0];
//        inputs.poseAmbiguity = 1;
//        inputs.distancesFromTags = new double[0];
//    }
//
}
