package frc.robot.subsystems.index;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
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
    private TalonFX m_indexMainMotor = new TalonFX(30, "*");
    private TalonFX m_indexSecondaryMotor = new TalonFX(31, "*");

    private SmartDashboardNumber mainIndexSpeed = new SmartDashboardNumber("index/index-main-speed", 0.15);
    private SmartDashboardNumber acceleratedMainIndexSpeed = new SmartDashboardNumber("index/index-main-accelerated-speed", 0.6);
    private SmartDashboardNumber reverseMainIndexSpeed = new SmartDashboardNumber("index/index-main-reverse-speed", -0.15);

    private SmartDashboardNumber secondaryIndexSpeed = new SmartDashboardNumber("index/index-secondary-speed", 0.9);
    private SmartDashboardNumber reverseSecondaryIndexSpeed = new SmartDashboardNumber("index/index-secondary-reverse-speed", -0.9);

    private CurrentLimitsConfigs mainIndexCurrentLimitsConfigs = new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(80)
            .withSupplyCurrentLimitEnable(true)
            .withStatorCurrentLimit(120)
            .withStatorCurrentLimitEnable(true);

    private CurrentLimitsConfigs secondaryIndexCurrentLimitsConfigs = new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(80)
            .withSupplyCurrentLimitEnable(true)
            .withStatorCurrentLimit(120)
            .withStatorCurrentLimitEnable(true);

    
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
                .withInverted(InvertedValue.Clockwise_Positive)
                .withPeakForwardDutyCycle(1d)
                .withPeakReverseDutyCycle(-1d)
                .withNeutralMode(NeutralModeValue.Brake)
        );

        this.m_indexMainMotor.getConfigurator().apply(mainIndexCurrentLimitsConfigs);
        this.m_indexSecondaryMotor.getConfigurator().apply(secondaryIndexCurrentLimitsConfigs);
    }

    public void startMainIndex() {
        this.m_indexMainMotor.setControl(new DutyCycleOut(mainIndexSpeed.getNumber()));
    }
    
    public void accelerateMainIndex() {
        this.m_indexMainMotor.setControl(new DutyCycleOut(acceleratedMainIndexSpeed.getNumber()));
    }

    public void reverseMainIndex() {
        this.m_indexMainMotor.setControl(new DutyCycleOut(reverseMainIndexSpeed.getNumber()));
    }

    public void stopMainIndex() {
        this.m_indexMainMotor.setControl(new DutyCycleOut(0));
    }

    public void startSecondaryIndex() {
        this.m_indexSecondaryMotor.setControl(new DutyCycleOut(secondaryIndexSpeed.getNumber()));
    }

    public void reverseSecondaryIndex() {
        this.m_indexSecondaryMotor.setControl(new DutyCycleOut(reverseSecondaryIndexSpeed.getNumber()));
    }

    public void stopSecondaryIndex() {
        this.m_indexSecondaryMotor.setControl(new DutyCycleOut(0));
    }

    public Command startMainIndexCommand() {
        return Commands.runOnce(this::startMainIndex, this);
    }

    public Command accelerateMainIndexCommand() {
        return Commands.runOnce(this::accelerateMainIndex, this);
    }

    public Command reverseMainIndexCommand() {
        return Commands.runOnce(this::reverseMainIndex, this);
    }

    public Command stopMainIndexCommand() {
        return Commands.runOnce(this::stopMainIndex, this);
    }

    public Command startSecondaryIndexCommand() {
        return Commands.runOnce(this::startSecondaryIndex, this);
    }

    public Command reverseSecondaryIndexCommand() {
        return Commands.runOnce(this::reverseSecondaryIndex, this);
    }

    public Command stopSecondaryIndexCommand() {
        return Commands.runOnce(this::stopSecondaryIndex, this);
    }


    public static Index getInstance() {
        if (instance == null) instance = new Index();
        return instance;
    }
}

