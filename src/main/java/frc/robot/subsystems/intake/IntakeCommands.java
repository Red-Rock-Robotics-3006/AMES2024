package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;

public class IntakeCommands {
    private static Intake intake = Intake.getInstance();

    public static Command stowIntake() {
        return new FunctionalCommand(
            () -> intake.startStow(), // chang,
            () -> {},
            (interrupted) -> intake.setPivotMotorSpeeds(0),
            () -> intake.atCurrentSpike() || intake.stowed(),
            intake
        ).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }

    public static Command deployIntake() {
        return new FunctionalCommand(
            () -> intake.startDeploy(), // chang,
            () -> {},
            (interrupted) -> intake.setPivotMotorSpeeds(0),
            () -> intake.atCurrentSpike() || intake.deployed(),
            intake
        ).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }

    public static Command enableIntake() {
        return new InstantCommand(
            () -> intake.enableIntake(),
            intake
        ); 
    }

    public static Command disableIntake() {
        return new InstantCommand(
            () -> intake.disableIntake(),
            intake
        );
    }
}
