package frc.robot.subsystems.index;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils3006.SmartDashboardNumber;

public class Index extends SubsystemBase{
    private static Index instance = null;
    private TalonFX m_indexMainMotor = new TalonFX(30);
    private TalonFX m_indexSecondaryMotor = new TalonFX(31);

    private SmartDashboardNumber mainIndexSpeed = new SmartDashboardNumber("index/index-main-speed", 0.5);
    private SmartDashboardNumber secondaryIndexSpeed = new SmartDashboardNumber("index/index-secondary-speed", 0.9);

    private Index() {
        super("index");

        this.m_indexMainMotor.getConfigurator().apply(
            new MotorOutputConfigs()
                .withInverted(InvertedValue.CounterClockwise_Positive)
                .withPeakForwardDutyCycle(1d)
                .withPeakReverseDutyCycle(-1d)
                .withNeutralMode(NeutralModeValue.Brake)
        );

        this.m_indexSecondaryMotor.getConfigurator().apply(
            new MotorOutputConfigs()
                .withInverted(InvertedValue.CounterClockwise_Positive)
                .withPeakForwardDutyCycle(1d)
                .withPeakReverseDutyCycle(-1d)
                .withNeutralMode(NeutralModeValue.Brake)
        );
    }

    public void startMainIndex() {
        this.m_indexMainMotor.setControl(new DutyCycleOut(mainIndexSpeed.getNumber()));
    }

    public void stopMainIndex() {
        this.m_indexMainMotor.setControl(new DutyCycleOut(0));
    }

    public void startSecondaryIndex() {
        this.m_indexSecondaryMotor.setControl(new DutyCycleOut(secondaryIndexSpeed.getNumber()));
    }

    public void stopSecondaryIndex() {
        this.m_indexSecondaryMotor.setControl(new DutyCycleOut(0));
    }

    public Command startMainIndexCommand() {
        return Commands.runOnce(this::startMainIndex, this);
    }

    public Command stopMainIndexCommand() {
        return Commands.runOnce(this::stopMainIndex, this);
    }

    public Command startSecondaryIndexCommand() {
        return Commands.runOnce(this::startSecondaryIndex, this);
    }

    public Command stopSecondaryIndexCommand() {
        return Commands.runOnce(this::stopSecondaryIndex, this);
    }


    public static Index getInstance() {
        if (instance == null) instance = new Index();
        return instance;
    }
}

