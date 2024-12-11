package frc.robot.subsystems.intake;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils3006.SmartDashboardNumber;

public class Intake extends SubsystemBase {
    private static Intake instance = null;

    private final TalonFX m_slapLeft = new TalonFX(21, "*"); // change motor ID
    private final TalonFX m_slapRight = new TalonFX(22, "*"); // change motor ID
    private final TalonFX m_intake = new TalonFX(20, "*"); // change motor ID

    private Slot0Configs pivotSlot0Configs = new Slot0Configs();
    private Slot0Configs intakeSlot0Configs = new Slot0Configs();

    private MotionMagicConfigs pivotMotionConfigs = new MotionMagicConfigs();
    private MotionMagicConfigs intakeMotionConfigs = new MotionMagicConfigs();

    private SmartDashboardNumber pivotMotionAccel = new SmartDashboardNumber("pivot/pivot-mm-accel", 175);
    private SmartDashboardNumber pivotMotionVelo = new SmartDashboardNumber("pivot/pivot-mm-velo", 75);

    private SmartDashboardNumber intakeMotionAccel = new SmartDashboardNumber("intake/intake-mm-accel", 30);

    private SmartDashboardNumber intakeKs = new SmartDashboardNumber("intake/ks", 0);
    private SmartDashboardNumber intakeKa = new SmartDashboardNumber("intake/ka", 0);
    private SmartDashboardNumber intakeKv = new SmartDashboardNumber("intake/kv", 0.1); // to be tuned;
    private SmartDashboardNumber intakeKp = new SmartDashboardNumber("intake/kp", 0);
    private SmartDashboardNumber intakeKi = new SmartDashboardNumber("intake/ki", 0);
    private SmartDashboardNumber intakeKd = new SmartDashboardNumber("intake/kd", 0);

    private SmartDashboardNumber pivotKs = new SmartDashboardNumber("pivot/ks", 0);
    private SmartDashboardNumber pivotKa = new SmartDashboardNumber("pivot/ka", 0);
    private SmartDashboardNumber pivotKv = new SmartDashboardNumber("pivot/kv", 0); // to be tuned;
    private SmartDashboardNumber pivotKp = new SmartDashboardNumber("pivot/kp", 1); // to be tuned;
    private SmartDashboardNumber pivotKi = new SmartDashboardNumber("pivot/ki", 0);
    private SmartDashboardNumber pivotKd = new SmartDashboardNumber("pivot/kd", 0);

    private SmartDashboardNumber intakeSpeed = new SmartDashboardNumber("intake/intake-speed", -2700);

    private SmartDashboardNumber pivotNormalizeSpeed = new SmartDashboardNumber("pivot/pivot-normalize-speed", -0.1);

    private SmartDashboardNumber pivotStowPosition = new SmartDashboardNumber("pivot/pivot-stow-position", 0.35);
    private SmartDashboardNumber pivotDeployPosition = new SmartDashboardNumber("pivot/pivot-deploy-position", 28.3);

    private SmartDashboardNumber pivotTolerance = new SmartDashboardNumber("pivot/pivot-tolerance", 0.1);
    private SmartDashboardNumber pivotSpikeThreshold = new SmartDashboardNumber("pivot/pivot-spike-threshold", 10);

    private Intake() {
        super("Intake");
        
        this.m_slapLeft.getConfigurator().apply(
            new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive)
                .withPeakForwardDutyCycle(1d)
                .withPeakReverseDutyCycle(-1d)
                .withNeutralMode(NeutralModeValue.Brake)
        );

        this.m_slapRight.getConfigurator().apply(
                new MotorOutputConfigs()
                        .withInverted(InvertedValue.Clockwise_Positive)
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

        pivotMotionConfigs = new MotionMagicConfigs()
            .withMotionMagicAcceleration(pivotMotionAccel.getNumber())
            .withMotionMagicCruiseVelocity(pivotMotionVelo.getNumber());

        intakeMotionConfigs = new MotionMagicConfigs()
            .withMotionMagicAcceleration(intakeMotionAccel.getNumber());

        this.m_slapLeft.getConfigurator().apply(pivotSlot0Configs);
        this.m_slapRight.getConfigurator().apply(pivotSlot0Configs);
        this.m_slapLeft.getConfigurator().apply(pivotMotionConfigs);
        this.m_slapRight.getConfigurator().apply(pivotMotionConfigs);
        this.m_intake.getConfigurator().apply(intakeSlot0Configs);
        this.m_intake.getConfigurator().apply(intakeMotionConfigs);

        this.m_slapRight.setControl(new Follower(21, true));
    }

    public void setStowPosition() {
        this.m_slapLeft.setControl(
            new MotionMagicVoltage(this.pivotStowPosition.getNumber()) // fix
                .withSlot(0)
                .withEnableFOC(true)
                .withOverrideBrakeDurNeutral(false)
        );
    }

    public void setDeployPosition() {
        this.m_slapLeft.setControl(
            new MotionMagicVoltage(this.pivotDeployPosition.getNumber()) // fix
                .withSlot(0)
                .withEnableFOC(true)
                .withOverrideBrakeDurNeutral(false)
        );
    }

    public void enableIntake() {
        this.m_intake.setControl(new MotionMagicVelocityVoltage(intakeSpeed.getNumber() / 60d) // change
                .withSlot(0)
                .withEnableFOC(true)
                .withOverrideBrakeDurNeutral(true));
    }

    public void disableIntake() {
        this.m_intake.setControl(new MotionMagicVelocityVoltage(0) // change
                .withSlot(0)
                .withEnableFOC(true)
                .withOverrideBrakeDurNeutral(true));
    }

    public void setNormalizeSpeed() {
        this.m_slapLeft.setControl(new DutyCycleOut(this.pivotNormalizeSpeed.getNumber()));
    }

    public void resetPivots() {
        this.m_slapLeft.setControl(new CoastOut());
        this.m_slapLeft.setPosition(0);
        this.m_slapRight.setPosition(0);
        this.m_slapLeft.setControl(new DutyCycleOut(0));
    }

    public boolean atCurrentSpike() { // TODO
        return Math.abs(this.m_slapLeft.getTorqueCurrent().getValueAsDouble()) > this.pivotSpikeThreshold.getNumber() ||
                Math.abs(this.m_slapRight.getTorqueCurrent().getValueAsDouble()) > this.pivotSpikeThreshold.getNumber();
    }

    @Override
    public void periodic() {
        if (pivotKs.hasChanged()
        || pivotKv.hasChanged()
        || pivotKp.hasChanged()
        || pivotKi.hasChanged()
        || pivotKd.hasChanged()
        || pivotKa.hasChanged()) {
            pivotSlot0Configs.kS = pivotKs.getNumber();
            pivotSlot0Configs.kV = pivotKv.getNumber();
            pivotSlot0Configs.kP = pivotKp.getNumber();
            pivotSlot0Configs.kI = pivotKi.getNumber();
            pivotSlot0Configs.kD = pivotKd.getNumber();
            pivotSlot0Configs.kA = pivotKa.getNumber();

            if (!Utils.isSimulation()) {this.m_slapLeft.getConfigurator().apply(pivotSlot0Configs); this.m_slapRight.getConfigurator().apply(pivotSlot0Configs);}
            System.out.println("applyied");
        }

        if (pivotMotionAccel.hasChanged() || pivotMotionVelo.hasChanged()) {
            pivotMotionConfigs.MotionMagicAcceleration = pivotMotionAccel.getNumber();
            pivotMotionConfigs.MotionMagicCruiseVelocity = pivotMotionVelo.getNumber();
            this.m_slapLeft.getConfigurator().apply(pivotMotionConfigs);
            this.m_slapRight.getConfigurator().apply(pivotMotionConfigs);
        }

        if (intakeKs.hasChanged()
        || intakeKv.hasChanged()
        || intakeKp.hasChanged()
        || intakeKi.hasChanged()
        || intakeKd.hasChanged()
        || intakeKa.hasChanged()) {
            intakeSlot0Configs.kS = intakeKs.getNumber();
            intakeSlot0Configs.kV = intakeKv.getNumber();
            intakeSlot0Configs.kP = intakeKp.getNumber();
            intakeSlot0Configs.kI = intakeKi.getNumber();
            intakeSlot0Configs.kD = intakeKd.getNumber();
            intakeSlot0Configs.kA = intakeKa.getNumber();

            if (!Utils.isSimulation()) this.m_intake.getConfigurator().apply(intakeSlot0Configs);
            System.out.println("applyied");
        }

        if (intakeMotionAccel.hasChanged()) {
            this.intakeMotionConfigs.MotionMagicAcceleration = intakeMotionAccel.getNumber();
            this.m_intake.getConfigurator().apply(intakeMotionConfigs);
        }

        SmartDashboard.putNumber("intake/intake-left-position", m_slapLeft.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("intake/intake-right-position", m_slapRight.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("intake/intake-left-spike", m_slapLeft.getTorqueCurrent().getValueAsDouble());
        SmartDashboard.putNumber("intake/intake-right-spike", m_slapRight.getTorqueCurrent().getValueAsDouble());
        SmartDashboard.putNumber("intake/intake-left-velocity", m_slapLeft.getVelocity().getValueAsDouble());

        SmartDashboard.putNumber("intake/intake-velo", m_intake.getVelocity().getValueAsDouble());

    }

    public Command enableIntakeCommand() {
        return Commands.runOnce(
            () -> this.enableIntake(), this
        );
    }

    public Command disableIntakeCommand() {
        return Commands.runOnce(
            () -> this.disableIntake(), this
        );
    }

    public Command setStowPositionCommand() {
        return Commands.runOnce(
            this::setStowPosition, this
        );
    }

    public Command setDeployPositionCommand() {
        return Commands.runOnce(
            this::setDeployPosition, this);
    }

    public Command normalizePivotCommand() {
        return new FunctionalCommand(
            () -> this.setNormalizeSpeed(), 
            () -> {}, 
            (interrupted) -> this.resetPivots(), 
            () -> this.atCurrentSpike(), this);
    }

    public static Intake getInstance() {
        if (Intake.instance == null)
            Intake.instance = new Intake();
        return Intake.instance;
    }
}
