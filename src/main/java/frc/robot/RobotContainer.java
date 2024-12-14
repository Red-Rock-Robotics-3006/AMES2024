// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.index.Index;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Turret;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;

public class RobotContainer {
  private static RobotContainer instance = null;

  private double MaxSpeed = CommandSwerveDrivetrain.kDriveMaxSpeed; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = CommandSwerveDrivetrain.kTurnMaxSpeed; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController drivestick = new CommandXboxController(0); // My drivestick
  private final CommandXboxController mechstick = new CommandXboxController(1);
  private final CommandSwerveDrivetrain drivetrain = CommandSwerveDrivetrain.getInstance(); // My drivetrain

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * CommandSwerveDrivetrain.kDriveDeadBand).withRotationalDeadband(MaxAngularRate * CommandSwerveDrivetrain.kTurnDeadBand) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final SwerveRequest.FieldCentricFacingAngle driveFacingAngle = new SwerveRequest.FieldCentricFacingAngle()
      .withDeadband(MaxSpeed * CommandSwerveDrivetrain.kDriveDeadBand).withRotationalDeadband(MaxAngularRate * CommandSwerveDrivetrain.kTurnDeadBand)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private final Telemetry logger = new Telemetry(MaxSpeed);

  private Shooter shooter = Shooter.getInstance();
  private Turret turret = Turret.getInstance();
  private Intake intake = Intake.getInstance();
  private Index index = Index.getInstance();

  private SendableChooser<Command> m_chooser = new SendableChooser<>();

  private void configureBindings() {
    this.configureSwerveBindings();
    this.configureMechBindings();

    // RobotModeTriggers.teleop().onTrue(
    //   Commands.parallel(
    //     shooter.normalizeHoodCommand(),
    //     turret.normalizeTurretCommand()
    //   )
    // );

    // mechstick.a().onTrue(
    //   turret.normalizeTurretCommand()
    // );

    // mechstick.b().onTrue(
    //   shooter.normalizeHoodCommand()
    // );

    // mechstick.x().onTrue(
    //   intake.normalizePivotCommand()
    // );

    // mechstick.y().onTrue(
    //   new InstantCommand(() -> shooter.setRequestedRPM(), shooter)
    // ).onFalse(
    //   new InstantCommand(() -> shooter.setShooterRPM(0), shooter)
    // );

    // mechstick.povUp().onTrue(
    //   Commands.runOnce(
    //     () -> {shooter.setFenderShotState(); turret.setTurretPosition(new Rotation2d());}, 
    //     shooter, turret)
    // );

    // mechstick.povLeft().onTrue(
    //   intake.setDeployPositionCommand()
    // );

    // mechstick.povRight().onTrue(
    //   intake.setStowPositionCommand()
    // );

    // mechstick.leftBumper().onTrue(
    //   Commands.runOnce(
    //     () -> {index.startMainIndex(); index.startSecondaryIndex();}, index)
    // );

    // mechstick.rightBumper().onTrue(
    //   Commands.runOnce(
    //     () -> {index.stopMainIndex(); index.stopSecondaryIndex();}, index)
    // );





    ;
    // mechstick.back().onTrue(intake.enableIntakeCommand());
    // mechstick.start().onTrue(intake.disableIntakeCommand());

    mechstick.a().onTrue(index.startMainIndexCommand());
    mechstick.b().onTrue(index.stopMainIndexCommand());
    mechstick.y().onTrue(intake.normalizePivotCommand());

    SmartDashboard.putNumber("autos/wait time", 6.5);
  }

  private void configureMechBindings() {
    RobotModeTriggers.teleop().onTrue(
      Commands.parallel(
        shooter.normalizeHoodCommand(),
        turret.normalizeTurretCommand(),
        intake.normalizePivotCommand()
      )
    );

    new Trigger(() -> drivestick.getHID().getRightTriggerAxis() > 0.25).onTrue(
      new FunctionalCommand(
        () -> {index.startSecondaryIndex();}, 
        () -> shooter.setRequestedRPM(), 
        (interrupted) -> {shooter.setShooterRPM(0); index.stopSecondaryIndex();},
        () -> !(drivestick.getHID().getRightTriggerAxis() > 0.25), shooter)
    );

    drivestick.a().onTrue(Commands.sequence(intake.enableIntakeCommand(), intake.setDeployPositionCommand()));
    drivestick.b().onTrue(Commands.sequence(intake.disableIntakeCommand(), intake.setStowPositionCommand()));

    drivestick.x().onTrue(new InstantCommand(() -> {shooter.disableAutoAim(); turret.disableAutoAim(); shooter.setFenderShotState(); turret.setTurretPosition(new Rotation2d());}, shooter, turret));
    drivestick.y().onTrue(new InstantCommand(() -> {shooter.disableAutoAim();turret.disableAutoAim(); shooter.setLowShotState(); turret.setTurretPosition(new Rotation2d());}, shooter, turret));

    drivestick.leftBumper().onTrue(new InstantCommand(() -> {shooter.enableAutoAim(); turret.enableAutoAim();}, shooter, turret));

    
  }

  private void configureSwerveBindings() {
    // drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
    //     drivetrain.applyRequest(() -> drive.withVelocityX(-drivestick.getLeftY() * MaxSpeed) // Drive forward with
    //                                                                                        // negative Y (forward)
    //         .withVelocityY(-drivestick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
    //         .withRotationalRate(-drivestick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
    //     ));

    drivetrain.setDefaultCommand(
      drivetrain.applyRequest(
        () -> {
          if (!drivetrain.getUseHeadingPID() || Math.abs(drivestick.getRightX()) > drivetrain.getTurnDeadBand()) {
            return drive.withVelocityX(-drivestick.getLeftY() * MaxSpeed)
                        .withVelocityY(-drivestick.getLeftX() * MaxSpeed)
                        .withRotationalRate(-drivestick.getRightX() * MaxAngularRate);
          }
          else {
            return driveFacingAngle.withVelocityX(-drivestick.getLeftY() * MaxSpeed)
                  .withVelocityY(-drivestick.getLeftX() * MaxSpeed)
                  .withTargetDirection(Rotation2d.fromDegrees(drivetrain.getTargetHeadingDegrees()));
            
          }
        }
      )
    );

    new Trigger(
      () -> Math.abs(drivestick.getRightX()) > drivetrain.getTurnDeadBand()
    ).onTrue(
      new FunctionalCommand(
        () -> {},
        () -> {drivetrain.setTargetHeadingDegrees(drivetrain.getHeadingDegrees());}, 
        (interrupted) -> {drivetrain.setTargetHeadingDegrees(drivetrain.getHeadingDegrees());}, 
        () -> !drivetrain.isRotating() && Math.abs(drivestick.getRightX()) < drivetrain.getTurnDeadBand())
    );
    
    // drivestick.leftBumper().onTrue(
    //   new InstantCommand(() -> drivetrain.toggleHeadingPID(), drivetrain)
    // );

    // drivestick.povLeft().onTrue(
    //   new InstantCommand(() -> drivetrain.setTargetHeadingDegrees(90), drivetrain)
    // );

    // drivestick.povUp().onTrue(
    //   new InstantCommand(() -> drivetrain.setTargetHeadingDegrees(0), drivetrain)
    // );

    // drivestick.povRight().onTrue(
    //   new InstantCommand(() -> drivetrain.setTargetHeadingDegrees(-90), drivetrain)
    // );

    // drivestick.povDown().onTrue(
    //   new InstantCommand(() -> drivetrain.setTargetHeadingDegrees(180), drivetrain)
    // );

    // mechstick.x().whileTrue(drivetrain.applyRequest(() -> brake));
    // mechstick.b().whileTrue(drivetrain
    //     .applyRequest(() -> point.withModuleDirection(new Rotation2d(-drivestick.getLeftY(), -drivestick.getLeftX()))));

    drivestick.start().onTrue(drivetrain.resetHeadingCommand());

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);
  }

  private void configureSelector() {
    m_chooser.setDefaultOption("NO AUTO", Commands.print("womp womp lmao"));

    m_chooser.addOption("TESTAUTO1", Autos.m_testAuto1());
    m_chooser.addOption("TESTAUTO1 PATHS", Autos.m_testAuto1Path());

    SmartDashboard.putData("Auto Chooser", m_chooser);
  }

  public double getAutoWaitTime() {
    return SmartDashboard.getNumber("autos/wait time", 0);
  }

  public RobotContainer() {
    drivetrain.setSwerveRequest(this.driveFacingAngle);
    configureBindings();
    configureSelector();
  }

  public void loop(){
    MaxSpeed = drivetrain.getMaxDriveSpeed();
    MaxAngularRate = drivetrain.getMaxTurnSpeed() * Math.PI;

    drive.Deadband = drivetrain.getDriveDeadBand();
    drive.RotationalDeadband = drivetrain.getTurnDeadBand();

    driveFacingAngle.Deadband = drivetrain.getDriveDeadBand();
    driveFacingAngle.RotationalDeadband = drivetrain.getTurnDeadBand();
  }

  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }

  public static RobotContainer getInstance() {
    if (instance == null) instance = new RobotContainer();
    return instance;
  }
}
