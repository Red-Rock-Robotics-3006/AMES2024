package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils3006.SmartDashboardNumber;

public class Intake extends SubsystemBase {
    private static Intake instance = null;

    private final TalonFX m_slapLeft = new TalonFX(0); // change motor ID
    private final TalonFX m_slapRight = new TalonFX(0); // change motor ID
    private final TalonFX m_intake = new TalonFX(0); // change motor ID

    private Slot0Configs pivotSlot0Configs;
    private Slot0Configs intakeSlot0Configs;

    private double stowPosition;
    private double deployPosition;
    private double tolerance = 10000; // fix

    private SmartDashboardNumber intakeKs = new SmartDashboardNumber("intake/ks", 0);
    private SmartDashboardNumber intakeKa = new SmartDashboardNumber("intake/ka", 0);
    private SmartDashboardNumber intakeKv = new SmartDashboardNumber("intake/kv", 0); // to be tuned;
    private SmartDashboardNumber intakeKp = new SmartDashboardNumber("intake/kp", 0);
    private SmartDashboardNumber intakeKi = new SmartDashboardNumber("intake/ki", 0);
    private SmartDashboardNumber intakeKd = new SmartDashboardNumber("intake/kd", 0);

    private SmartDashboardNumber pivotKs = new SmartDashboardNumber("pivot/ks", 0);
    private SmartDashboardNumber pivotKa = new SmartDashboardNumber("pivot/ka", 0);
    private SmartDashboardNumber pivotKv = new SmartDashboardNumber("pivot/kv", 0); // to be tuned;
    private SmartDashboardNumber pivotKp = new SmartDashboardNumber("pivot/kp", 0); // to be tuned;
    private SmartDashboardNumber pivotKi = new SmartDashboardNumber("pivot/ki", 0);
    private SmartDashboardNumber pivotKd = new SmartDashboardNumber("pivot/kd", 0);

    private Intake() {
        super("Intake");

        this.m_slapRight.getConfigurator().apply(
                new MotorOutputConfigs()
                        .withInverted(InvertedValue.Clockwise_Positive)
                        .withPeakForwardDutyCycle(1d)
                        .withPeakReverseDutyCycle(-1d)
                        .withNeutralMode(NeutralModeValue.Brake));

        this.m_slapLeft.getConfigurator().apply(
                new MotorOutputConfigs()
                        .withInverted(InvertedValue.CounterClockwise_Positive)
                        .withPeakForwardDutyCycle(1d)
                        .withPeakReverseDutyCycle(-1d)
                        .withNeutralMode(NeutralModeValue.Brake));

        this.m_intake.getConfigurator().apply(
                new MotorOutputConfigs()
                        .withInverted(InvertedValue.Clockwise_Positive)
                        .withPeakForwardDutyCycle(1d)
                        .withPeakReverseDutyCycle(-1d)
                        .withNeutralMode(NeutralModeValue.Brake));

        this.pivotSlot0Configs = new Slot0Configs()
                .withKS(pivotKs.getNumber())
                .withKA(pivotKa.getNumber())
                .withKV(pivotKv.getNumber())
                .withKP(pivotKp.getNumber())
                .withKI(pivotKi.getNumber())
                .withKD(pivotKd.getNumber());

        this.intakeSlot0Configs = new Slot0Configs()
                .withKS(intakeKs.getNumber())
                .withKA(intakeKa.getNumber())
                .withKV(intakeKv.getNumber())
                .withKP(intakeKp.getNumber())
                .withKI(intakeKi.getNumber())
                .withKD(intakeKd.getNumber());

        this.m_slapLeft.getConfigurator().apply(pivotSlot0Configs);
        this.m_slapRight.getConfigurator().apply(pivotSlot0Configs);
        this.m_intake.getConfigurator().apply(intakeSlot0Configs);
    }

    public void startStow() {
        this.m_slapRight.setControl(
            new PositionVoltage(0) // fix
                .withSlot(0)
                .withEnableFOC(true)
                .withOverrideBrakeDurNeutral(false)
        );
        this.m_slapLeft.setControl(
            new PositionVoltage(0) // fix
                .withSlot(0)
                .withEnableFOC(true)
                .withOverrideBrakeDurNeutral(false)
        );
    }

    public void startDeploy() {
        this.m_slapRight.setControl(
            new PositionVoltage(10000) // fix
                .withSlot(0)
                .withEnableFOC(true)
                .withOverrideBrakeDurNeutral(false)
        );
        this.m_slapLeft.setControl(
            new PositionVoltage(10000) // fix
                .withSlot(0)
                .withEnableFOC(true)
                .withOverrideBrakeDurNeutral(false)
        );
    }

    public void enableIntake() {
        this.m_intake.setControl(new MotionMagicVelocityVoltage(10000) // change
                .withSlot(0)
                .withEnableFOC(true));
    }

    public void disableIntake() {
        this.m_intake.setControl(new MotionMagicVelocityVoltage(0) // change
                .withSlot(0)
                .withEnableFOC(true));
    }

    public void setPivotMotorSpeeds(double speed) {
        m_slapLeft.set(speed);
        m_slapRight.set(speed);
    }

    public boolean atCurrentSpike() { // TODO
        return false;
    }

    public boolean stowed() {
        return Math.abs(m_slapLeft.getPosition().getValue() - stowPosition) < tolerance && Math.abs(m_slapRight.getPosition().getValue() - stowPosition) < tolerance;
    }

    public boolean deployed() {
        return Math.abs(m_slapLeft.getPosition().getValue() - deployPosition) < tolerance && Math.abs(m_slapRight.getPosition().getValue() - deployPosition) < tolerance;
    }

    public static Intake getInstance() {
        if (Intake.instance == null)
            Intake.instance = new Intake();
        return Intake.instance;
    }
}
