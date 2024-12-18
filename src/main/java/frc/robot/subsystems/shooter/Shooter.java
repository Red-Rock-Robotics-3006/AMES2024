package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils3006.SmartDashboardNumber;
import frc.robot.vision.Localization;

public class Shooter extends SubsystemBase{
    private static Shooter instance = null;

    public static double kMaxHoodAngle = 85,
                         kMinHoodAngle = 50;
    public static double kMaxHoodRotation = 1,
                         kMinHoodRotation = 0;

    private TalonFX m_shooterMotor = new TalonFX(50, "*");
    private TalonFX m_hoodMotor = new TalonFX(51, "*");

    private Slot0Configs shooterSlot0Configs = new Slot0Configs();
    private Slot0Configs hoodSlot0Configs = new Slot0Configs();

    private MotionMagicConfigs shooterMotionMagicConfigs = new MotionMagicConfigs();
    private MotionMagicConfigs hoodMotionMagicConfigs = new MotionMagicConfigs();

    private CurrentLimitsConfigs shooterCurrentLimitsConfigs = new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(80)
            .withSupplyCurrentLimitEnable(true)
            .withStatorCurrentLimit(120)
            .withStatorCurrentLimitEnable(true);

    private CurrentLimitsConfigs hoodCurrentLimitsConfigs = new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(80)
            .withSupplyCurrentLimitEnable(true)
            .withStatorCurrentLimit(120)
            .withStatorCurrentLimitEnable(true);

    private SmartDashboardNumber shooterAccel = new SmartDashboardNumber("shooter/shooter-accel-motion-magic", 75);

    private SmartDashboardNumber hoodMotionAccel = new SmartDashboardNumber("hood/hood-mm-accel", 40);
    private SmartDashboardNumber hoodMotionVelo = new SmartDashboardNumber("hood/hood-mm-velo", 40);

    private SmartDashboardNumber shooterKs = new SmartDashboardNumber("shooter/ks", 0);
    private SmartDashboardNumber shooterKa = new SmartDashboardNumber("shooter/ka", 0);
    private SmartDashboardNumber shooterKv = new SmartDashboardNumber("shooter/kv", 0.133); //to be tuned;
    private SmartDashboardNumber shooterKp = new SmartDashboardNumber("shooter/kp", 0.4); //to be tuned;
    private SmartDashboardNumber shooterKi = new SmartDashboardNumber("shooter/ki", 0);
    private SmartDashboardNumber shooterKd = new SmartDashboardNumber("shooter/kd", 0);

    private SmartDashboardNumber hoodKs = new SmartDashboardNumber("hood/ks", 0);
    private SmartDashboardNumber hoodKa = new SmartDashboardNumber("hood/ka", 0);
    private SmartDashboardNumber hoodKv = new SmartDashboardNumber("hood/kv", 0); //to be tuned;
    private SmartDashboardNumber hoodKp = new SmartDashboardNumber("hood/kp", 7); //to be tuned;
    private SmartDashboardNumber hoodKi = new SmartDashboardNumber("hood/ki", 0);
    private SmartDashboardNumber hoodKd = new SmartDashboardNumber("hood/kd", 0);

    private SmartDashboardNumber fenderAngle = new SmartDashboardNumber("fender shot angle", 50);
    private SmartDashboardNumber fenderRPM = new SmartDashboardNumber("fender shot rpm", 1500);

    private SmartDashboardNumber lowAngle = new SmartDashboardNumber("low shot angle", 75);
    private SmartDashboardNumber lowRPM = new SmartDashboardNumber("low shot rpm", 400);

    private SmartDashboardNumber reverseRPM = new SmartDashboardNumber("reverse shot rpm", -750);

    private SmartDashboardNumber spikeThreshold = new SmartDashboardNumber("hood/hood-spike-threshold", 10.5);
    private SmartDashboardNumber normalizeSpeed = new SmartDashboardNumber("hood/hood-normalize-speed", -0.05);

    private SmartDashboardNumber pidTolerance = new SmartDashboardNumber("hood/hood-pid-tolerance", 0.1);
    private SmartDashboardNumber positionTolerance = new SmartDashboardNumber("hood/hood-position-tolerance", 0.1);//gyatt good googly moogly

    private double nonClampedTargetRevolution;
    private double requestedRPM;

    //for smartdashboard only
    private double targetHoodAngle;
    private double targetRPM;
    private double targetPosition;

    private boolean autoAimEnabled = false, onBlue;

    private Shooter() {
        super("Shooter");

        this.onBlue = !DriverStation.getAlliance().isPresent() || DriverStation.getAlliance().get() == DriverStation.Alliance.Blue;

        this.m_shooterMotor.getConfigurator().apply(
            new MotorOutputConfigs()
                .withInverted(InvertedValue.CounterClockwise_Positive)
                .withPeakForwardDutyCycle(1d)
                .withPeakReverseDutyCycle(-1d)
                .withNeutralMode(NeutralModeValue.Brake)
        );

        this.m_hoodMotor.getConfigurator().apply(
            new MotorOutputConfigs()
                .withInverted(InvertedValue.CounterClockwise_Positive)
                .withPeakForwardDutyCycle(1d)
                .withPeakReverseDutyCycle(-1d)
                .withNeutralMode(NeutralModeValue.Brake)
        );

        this.shooterSlot0Configs = new Slot0Configs()
            .withKS(shooterKs.getNumber())
            .withKA(shooterKa.getNumber())
            .withKV(shooterKv.getNumber())
            .withKP(shooterKp.getNumber())
            .withKI(shooterKi.getNumber())
            .withKD(shooterKd.getNumber());

        this.hoodSlot0Configs = new Slot0Configs()
            .withKS(hoodKs.getNumber())
            .withKA(hoodKa.getNumber())
            .withKV(hoodKv.getNumber())
            .withKP(hoodKp.getNumber())
            .withKI(hoodKi.getNumber())
            .withKD(hoodKd.getNumber());

        this.shooterMotionMagicConfigs = new MotionMagicConfigs()
            .withMotionMagicAcceleration(shooterAccel.getNumber());

        this.hoodMotionMagicConfigs = new MotionMagicConfigs()
            .withMotionMagicAcceleration(hoodMotionAccel.getNumber())
            .withMotionMagicCruiseVelocity(hoodMotionVelo.getNumber());

        this.m_shooterMotor.getConfigurator().apply(shooterSlot0Configs);
        this.m_shooterMotor.getConfigurator().apply(shooterMotionMagicConfigs);
        this.m_shooterMotor.getConfigurator().apply(shooterCurrentLimitsConfigs);
        this.m_hoodMotor.getConfigurator().apply(hoodSlot0Configs);
        this.m_hoodMotor.getConfigurator().apply(hoodMotionMagicConfigs);
        this.m_hoodMotor.getConfigurator().apply(hoodCurrentLimitsConfigs);
    }

    public void setHoodAngle(double angle) {
        this.m_hoodMotor.setControl(new MotionMagicVoltage(MathUtil.clamp(this.angleToRotation(angle), kMinHoodRotation + 0.07, kMaxHoodRotation))
                                        .withSlot(0)
                                        .withEnableFOC(true)
                                        .withOverrideBrakeDurNeutral(true)
        );
        this.targetHoodAngle = angle;
        this.targetPosition = this.angleToRotation(angle);
        this.nonClampedTargetRevolution = this.angleToRotation(angle);
    }

    public void setShooterRPM(double rpm) {
        this.m_shooterMotor.setControl(new MotionMagicVelocityVoltage(rpm / 60d)
                                        .withSlot(0)
                                        .withEnableFOC(true)
                                        .withOverrideBrakeDurNeutral(true)
        );
        this.targetRPM = rpm;
    }

    public void setRequestedRPM() {
        this.setShooterRPM(this.requestedRPM);
    }

    public void setFenderShotState() {
        this.setHoodAngle(fenderAngle.getNumber());
        this.requestedRPM = fenderRPM.getNumber();
    }

    public void setLowShotState() {
        this.setHoodAngle(lowAngle.getNumber());
        this.requestedRPM = lowRPM.getNumber();
    }

    public void setReverseRPM() {
        this.requestedRPM = reverseRPM.getNumber();
        this.setRequestedRPM();
    }

    public void resetHood() {
        this.m_hoodMotor.setControl(new CoastOut());
        this.m_hoodMotor.setPosition(0d);
    }

    public boolean inSpikeCurrent() {
        return Math.abs(this.m_hoodMotor.getTorqueCurrent().getValueAsDouble()) > this.spikeThreshold.getNumber();
    }

    private void setNormalizeSpeed() {
        this.m_hoodMotor.setControl(new DutyCycleOut(this.normalizeSpeed.getNumber()));
    }

    public void increaseFenderRPM() {
        this.fenderRPM.putNumber(this.fenderRPM.getNumber() + 100);
    }

    public void decreaseFenderRPM() {
        this.fenderRPM.putNumber(this.fenderRPM.getNumber() - 100);
    }

    public void increaseFenderAngle() {
        this.fenderAngle.putNumber(this.fenderAngle.getNumber() + 2.5);
    }

    public void decreaseFenderAngle() {
        this.fenderAngle.putNumber(this.fenderAngle.getNumber() - 2.5);
    }

    public void enableAutoAim() {
        this.autoAimEnabled = true;
    }

    public void disableAutoAim() {
        this.autoAimEnabled = false;
    }

    public boolean isReady() {
        return Math.abs(this.m_hoodMotor.getClosedLoopError().getValueAsDouble()) < this.pidTolerance.getNumber() &&
            Math.abs(this.m_hoodMotor.getClosedLoopReference().getValueAsDouble() - this.nonClampedTargetRevolution) < this.positionTolerance.getNumber();
    }

    private double angleToRotation(double angle) {
        return ((kMaxHoodRotation - kMinHoodRotation) / (kMaxHoodAngle - kMinHoodAngle)) * (angle - kMinHoodAngle) + kMinHoodRotation;
    }

    @Override
    public void periodic() {
        if (shooterKs.hasChanged()
        || shooterKv.hasChanged()
        || shooterKp.hasChanged()
        || shooterKi.hasChanged()
        || shooterKd.hasChanged()
        || shooterKa.hasChanged()) {
            shooterSlot0Configs.kS = shooterKs.getNumber();
            shooterSlot0Configs.kV = shooterKv.getNumber();
            shooterSlot0Configs.kP = shooterKp.getNumber();
            shooterSlot0Configs.kI = shooterKi.getNumber();
            shooterSlot0Configs.kD = shooterKd.getNumber();
            shooterSlot0Configs.kA = shooterKa.getNumber();

            if (!Utils.isSimulation()) this.m_shooterMotor.getConfigurator().apply(shooterSlot0Configs);
            System.out.println("applyied");
        }

        if (shooterAccel.hasChanged()) {
            shooterMotionMagicConfigs.MotionMagicAcceleration = shooterAccel.getNumber();
            this.m_shooterMotor.getConfigurator().apply(shooterMotionMagicConfigs);
        }

        SmartDashboard.putNumber("shooter/shooter-acceleration", this.m_shooterMotor.getAcceleration().getValueAsDouble());
        SmartDashboard.putNumber("shooter/shooter-velocity", this.m_shooterMotor.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("shooter/shooter-rpm-target", this.targetRPM);

        if (hoodKs.hasChanged()
            || hoodKv.hasChanged()
            || hoodKp.hasChanged()
            || hoodKi.hasChanged()
            || hoodKd.hasChanged()
            || hoodKa.hasChanged()) {
            hoodSlot0Configs.kS = hoodKs.getNumber();
            hoodSlot0Configs.kV = hoodKv.getNumber();
            hoodSlot0Configs.kP = hoodKp.getNumber();
            hoodSlot0Configs.kI = hoodKi.getNumber();
            hoodSlot0Configs.kD = hoodKd.getNumber();
            hoodSlot0Configs.kA = hoodKa.getNumber();

            if (!Utils.isSimulation()) this.m_hoodMotor.getConfigurator().apply(hoodSlot0Configs);
            System.out.println("applyied");
        }

        if (hoodMotionAccel.hasChanged() || hoodMotionVelo.hasChanged()) {
            hoodMotionMagicConfigs.MotionMagicAcceleration = this.hoodMotionAccel.getNumber();
            hoodMotionMagicConfigs.MotionMagicCruiseVelocity = this.hoodMotionVelo.getNumber();
            this.m_hoodMotor.getConfigurator().apply(this.hoodMotionMagicConfigs);
        }

        SmartDashboard.putNumber("hood/hood-target-position", this.targetPosition);
        SmartDashboard.putNumber("hood/hood-position", this.m_hoodMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("hood/hood-target-angle", this.targetHoodAngle);
        SmartDashboard.putNumber("hood/hood-torque-current", this.m_hoodMotor.getTorqueCurrent().getValueAsDouble());
        SmartDashboard.putBoolean("hood/hood-at-spike", this.inSpikeCurrent());
        SmartDashboard.putBoolean("hood/hood-is-ready", this.isReady());
        SmartDashboard.putBoolean("hood/hood-auto-aim-enabled", this.autoAimEnabled);

        SmartDashboard.putNumber("shooter/shooter-request-rpm", this.requestedRPM);
        if (!this.onBlue) {
            SmartDashboard.putNumber("distance to target", Localization.getDistanceToTargetRed());
        } else {
            SmartDashboard.putNumber("distance to target", Localization.getDistanceToTargetBlue());
        }
        if (this.autoAimEnabled) {
            if (this.onBlue) this.aimToTargetBlue();
            else this.aimToTargetRed(); 
        }
    }

    private void aimToTargetRed() {
        ShotParameter p = InterpolatingTable.getRed(Localization.getDistanceToTargetRed());
        this.setHoodAngle(p.pivotAngleDeg);
        this.requestedRPM = p.rpm;
    }

    private void aimToTargetBlue() {
        ShotParameter p = InterpolatingTable.getBlue(Localization.getDistanceToTargetBlue());
        this.setHoodAngle(p.pivotAngleDeg);
        this.requestedRPM = p.rpm;
    }

    public Command normalizeHoodCommand() {
        return new FunctionalCommand(
            () -> {
                this.disableAutoAim();
                this.setNormalizeSpeed();
            }, 
            () -> {}, 
            (interrupted) -> {
                this.resetHood();
            }, 
            () -> this.inSpikeCurrent(), 
            this);
    }

    public static Shooter getInstance() {
        if (instance == null) instance = new Shooter();
        return instance;
    }
}
