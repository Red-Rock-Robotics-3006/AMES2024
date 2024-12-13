package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.index.Index;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Turret;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;

public class Autos {
    private static CommandSwerveDrivetrain swerve = CommandSwerveDrivetrain.getInstance();
    private static Shooter shooter = Shooter.getInstance();
    private static Turret turret = Turret.getInstance();
    private static Intake intake = Intake.getInstance();
    private static Index index = Index.getInstance();
    private static RobotContainer robotContainer = RobotContainer.getInstance();

    public static Command m_testAuto1Path() {
        return swerve.getAuto("TestAuto1");
    }

    public static Command m_testAuto1() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> {turret.setTurretPosition(new Rotation2d()); shooter.setFenderShotState(); shooter.setRequestedRPM();}),
            Commands.sequence(intake.enableIntakeCommand(), intake.setDeployPositionCommand()),
            index.startMainIndexCommand(),
            swerve.getAuto("TestAuto1"),
            // new WaitCommand(5),
            new InstantCommand(() -> {index.startSecondaryIndex();})
        );
    }

    public static Command m_leaveShootAuto1() {
        return new SequentialCommandGroup(
            index.startMainIndexCommand(),
            new InstantCommand(() -> shooter.setRequestedRPM()),
            swerve.getAuto(""),
            new WaitCommand(robotContainer.getAutoWaitTime()),
            swerve.getAuto("")
        );
    }
}
