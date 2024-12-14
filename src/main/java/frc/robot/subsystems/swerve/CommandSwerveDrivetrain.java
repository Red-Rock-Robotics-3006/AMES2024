package frc.robot.subsystems.swerve;

import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.ForwardReference;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Utils3006.SmartDashboardBoolean;
import frc.robot.Utils3006.SmartDashboardNumber;
import frc.robot.subsystems.swerve.generated.TunerConstants;
import frc.robot.vision.LimelightHelpers;
import frc.robot.vision.Localization;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * subsystem so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
    public static final double kDriveMaxSpeed = 6;
    public static final double kTurnMaxSpeed = 3;

    public static final double kRotationOmegaSignificance = 1;

    public static final double kDriveDeadBand = 0.05;
    public static final double kTurnDeadBand = 0.05;

    public static final double kRotateP = 2;
    public static final double kRotateI = 0;
    public static final double kRotateD = 0.1;

    private SmartDashboardNumber rotateP, rotateI, rotateD;

    private SmartDashboardNumber rotationOmegaSignificance;
    private SmartDashboardNumber driveMaxSpeed;
    private SmartDashboardNumber turnMaxSpeed;
    private SmartDashboardNumber driveDeadBand;
    private SmartDashboardNumber turnDeadBand;

    private boolean enableHeadingPID = true;

    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private final Rotation2d BlueAlliancePerspectiveRotation = Rotation2d.fromDegrees(0);
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private final Rotation2d RedAlliancePerspectiveRotation = Rotation2d.fromDegrees(180);
    // private final Rotation2d RedAlliancePerspectiveRotation = Rotation2d.fromDegrees(0);

    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean hasAppliedOperatorPerspective = false;

    private double targetHeadingDegrees = 0;

    private SwerveRequest.FieldCentricFacingAngle angleRequest;

    private final SwerveRequest.ApplyChassisSpeeds autoRequest = new SwerveRequest.ApplyChassisSpeeds();

    private SmartDashboardNumber kRejectionDistance = new SmartDashboardNumber("localization/rejection-distance", 3);
    private SmartDashboardNumber kRejectionRotationRate = new SmartDashboardNumber("localization/rejection-rotation-rate", 3);

    private SmartDashboardBoolean visionEnabled = new SmartDashboardBoolean("localization/vision-enabled", true);

    private Field2d field = new Field2d();

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency, SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        initialize();
    }
    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        initialize();
    }

    private void initialize(){
        this.rotationOmegaSignificance = new SmartDashboardNumber("dt/rotation rate limit", kRotationOmegaSignificance);
        this.driveMaxSpeed = new SmartDashboardNumber("dt/max drive", kDriveMaxSpeed);
        this.turnMaxSpeed = new SmartDashboardNumber("dt/max turn", kTurnMaxSpeed);
        this.driveDeadBand = new SmartDashboardNumber("dt/drive deadband", kDriveDeadBand);
        this.turnDeadBand = new SmartDashboardNumber("dt/turn deadband", kTurnDeadBand);

        this.rotateP = new SmartDashboardNumber("dt/heading p", kRotateP);
        this.rotateI = new SmartDashboardNumber("dt/heading i", kRotateI);
        this.rotateD = new SmartDashboardNumber("dt/heading d", kRotateD);

        SmartDashboard.putData("dt/dt-field", this.field);

        this.configurePathPlanner();

        this.seedFieldRelative(
            new Pose2d(
                this.getPose().getX(),
                this.getPose().getY(),
                new Rotation2d()
            )
        );
    }

    private void configurePathPlanner() {
        double driveBaseRadius = 0;
        for (var moduleLocation : m_moduleLocations) {
            driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
        }

        AutoBuilder.configureHolonomic(
            ()-> this.getPose(), // Supplier of current robot pose
            this::seedFieldRelative,  // Consumer for seeding pose against auto
            this::getCurrentRobotChassisSpeeds,
            (speeds)->this.setControl(autoRequest.withSpeeds(speeds)), // Consumer of ChassisSpeeds to drive the robot
            new HolonomicPathFollowerConfig(new PIDConstants(10, 0, 0),
                                            new PIDConstants(10, 0, 0),
                                            TunerConstants.kSpeedAt12VoltsMps,
                                            driveBaseRadius, 
                                            new ReplanningConfig()),
            // ()->false, // Change this if the path needs to be flipped on red vs blue
            () -> {//TODO this is is testing and we hope it works
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                  return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            },
            this); // Subsystem for requirements
    }

    public void setSwerveRequest(SwerveRequest.FieldCentricFacingAngle request){
        this.angleRequest = request;
        // this.angleRequest.ForwardReference = ForwardReference.RedAlliance;
        angleRequest.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    @Override
    public void periodic() {
        /* Periodically try to apply the operator perspective */
        /* If we haven't applied the operator perspective before, then we should apply it regardless of DS state */
        /* This allows us to correct the perspective in case the robot code restarts mid-match */
        /* Otherwise, only check and apply the operator perspective if the DS is disabled */
        /* This ensures driving behavior doesn't change until an explicit disable event occurs during testing*/
        if (!hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent((allianceColor) -> {
                this.setOperatorPerspectiveForward(
                        allianceColor == Alliance.Red ? RedAlliancePerspectiveRotation
                                : BlueAlliancePerspectiveRotation);
                hasAppliedOperatorPerspective = true;
            });
        }

        // this.driveMaxSpeed = SmartDashboard.getNumber("dt/max drive", kDriveMaxSpeed);
        // this.turnMaxSpeed = SmartDashboard.getNumber("dt/max turn", kTurnMaxSpeed);
        // this.rotationOmegaSignificance = SmartDashboard.getNumber("dt/rotation rate limit", kRotationOmegaSignificance);
        // this.driveDeadBand = SmartDashboard.getNumber("dt/drive deadband", kDriveDeadBand);
        // this.turnDeadBand = SmartDashboard.getNumber("dt/turn deadband", kTurnDeadBand);

        // this.rotateP = SmartDashboard.getNumber("dt/heading p", kRotateP);
        // this.rotateI = SmartDashboard.getNumber("dt/heading i", kRotateI);
        // this.rotateD = SmartDashboard.getNumber("dt/heading d", kRotateD);

        this.angleRequest.HeadingController.setPID(this.rotateP.getNumber(), this.rotateI.getNumber(), this.rotateD.getNumber());
        
        SmartDashboard.putBoolean("dt/using heading pid", this.enableHeadingPID);
        SmartDashboard.putNumber("dt/current heading", this.getHeadingDegrees());
        SmartDashboard.putNumber("dt/target heading", this.getTargetHeadingDegrees());

        SmartDashboard.putBoolean("dt/hasAPpliedOperatorPerspectrive", hasAppliedOperatorPerspective);

        this.field.setRobotPose(this.getPose());

        if (visionEnabled.getValue()) updateVisionMeasurements();
    }

    public void updateVisionMeasurements() {
        for (Localization.LimeLightPoseEstimateWrapper estimateWrapper : Localization.getPoseEstimates(this.getHeadingDegrees())) {
            if (estimateWrapper.tiv && poseEstimateIsValid(estimateWrapper.poseEstimate)) {
                this.addVisionMeasurement(estimateWrapper.poseEstimate.pose,
                                        estimateWrapper.poseEstimate.timestampSeconds, 
                                        estimateWrapper.getStdvs(estimateWrapper.poseEstimate.avgTagDist));
                estimateWrapper.field.setRobotPose(
                    estimateWrapper.poseEstimate.pose
                );
            }
        }
    }

    private boolean poseEstimateIsValid(LimelightHelpers.PoseEstimate e) {
        return e.avgTagDist < kRejectionDistance.getNumber() && Math.abs(this.getRotationRateDegrees()) < kRejectionRotationRate.getNumber();
    }

    public Command resetHeadingCommand(){
        return new InstantCommand(
            () -> {
                this.seedFieldRelative(
                    new Pose2d(
                        this.getState().Pose.getX(),
                        this.getState().Pose.getY(),
                        new Rotation2d()
                    )
                );
                this.targetHeadingDegrees = 0;
            }
        );
    }

    public Pose2d getPose() {
        return this.getState().Pose;
    }

    public ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return this.getState().speeds;
    }

    public void setTargetHeadingDegrees(double degrees){
        this.targetHeadingDegrees = degrees;
    }

    public Command getAuto(String s) {
        return new PathPlannerAuto(s);
    }

    public double getHeadingDegrees(){
        return this.getState().Pose.getRotation().getDegrees();
    }

    public double getTargetHeadingDegrees(){
        return this.targetHeadingDegrees;
    }

    public boolean isRotating(){
        return Math.abs(this.getPigeon2().getRate()) > this.rotationOmegaSignificance.getNumber();
    }

    public double getMaxDriveSpeed(){
        return this.driveMaxSpeed.getNumber();
    }

    public double getMaxTurnSpeed(){
        return this.turnMaxSpeed.getNumber();
    }

    public double getDriveDeadBand(){
        return this.driveDeadBand.getNumber();
    }

    public double getTurnDeadBand(){
        return this.turnDeadBand.getNumber();
    }

    public void setUseHeadingPID(boolean b){
        this.enableHeadingPID = b;
    }

    public boolean getUseHeadingPID(){
        return this.enableHeadingPID;
    }

    public void toggleHeadingPID(){
        this.enableHeadingPID = !this.enableHeadingPID;
    }

    /**
     * Returns drivetrain heading PID coefficients in the form of a double array with array.length == 3
     * 
     * @return Drivetrain Heading PID coeffs
     */
    public double[] getHeadingPIDCoeffs(){
        return new double[]{this.rotateP.getNumber(), this.rotateI.getNumber(), this.rotateD.getNumber()};
    }

    public double getRotationRateDegrees() {
        return this.getPigeon2().getRate();
    }

    public static CommandSwerveDrivetrain getInstance(){
        return TunerConstants.DriveTrain;
    }
}
