package frc.robot.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Localization {
    private static int[] validIDs = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14};
    private static String[] limeLightNames = {"limelight-front", "limelight-back"};
    private static NetworkTable[] limelightTables;

    public static void initialize() {
        limelightTables = new NetworkTable[limeLightNames.length];
        int i = 0;
        for (String s : limeLightNames) {
            LimelightHelpers.SetFiducialIDFiltersOverride(s, validIDs);
            limelightTables[i] = NetworkTableInstance.getDefault().getTable(s);
            i++;
        }
    }

    public static LimeLightPoseEstimateWrapper[] getPoseEstimates(double headingDegrees) {
        LimeLightPoseEstimateWrapper[] estimates = new LimeLightPoseEstimateWrapper[limeLightNames.length];
        int i = 0;
        for (String s : limeLightNames) {
            LimelightHelpers.SetRobotOrientation(s, headingDegrees, 0, 0, 0, 0, 0);
            estimates[i] = new LimeLightPoseEstimateWrapper();

        }
        return estimates;
    }
    //TODO
    public static double getDistanceToTargetRed() {
        return 0;
    }

    //TODO
    public static double getDistanceToTargetBlue() {
        return 0;
    }

    //TODO
    public static Pose2d getPose2d() {
        return new Pose2d();
    }

    //TODO
    public static Rotation2d getAngleToRed() {
        return new Rotation2d();
    }

    //TODO
    public static Rotation2d getAngleToBlue() {
        return new Rotation2d();
    }

    public static class LimeLightPoseEstimateWrapper {
        public LimelightHelpers.PoseEstimate poseEstimate;
        public String name;
        public double[] stdvs;

        public Matrix<N3, N1> getStdvs() {
            return VecBuilder.fill(
                stdvs[0],
                stdvs[1],
                stdvs[2]
            );
        }

        public LimeLightPoseEstimateWrapper withName(String name) {
            this.name = name;
            return this;
        }

        public LimeLightPoseEstimateWrapper withPoseEstimate(LimelightHelpers.PoseEstimate estimate) {
            this.poseEstimate = estimate;
            return this;
        }
    }
}
