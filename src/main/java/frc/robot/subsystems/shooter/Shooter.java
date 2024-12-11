package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;

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

    public static double kMaxHoodAngle, 
                         kMinHoodAngle;
    public static double kMaxHoodRotation, 
                         kMinHoodRotation;

    private TalonFX m_shooterMotor = new TalonFX(50);
    private TalonFX m_hoodMotor = new TalonFX(51);

    private Slot0Configs shooterSlot0Configs = new Slot0Configs();
    private Slot0Configs hoodSlot0Configs = new Slot0Configs();

    private MotionMagicConfigs shooterMotionMagicConfigs = new MotionMagicConfigs();

    private SmartDashboardNumber shooterAccel = new SmartDashboardNumber("shooter/shooter-accel-motion-magic", 100);

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

    private SmartDashboardNumber fenderAngle = new SmartDashboardNumber("fender shot angle", 0);
    private SmartDashboardNumber fenderRPM = new SmartDashboardNumber("fender shot rpm", 0);

    private SmartDashboardNumber lowAngle = new SmartDashboardNumber("low shot angle", 0);
    private SmartDashboardNumber lowRPM = new SmartDashboardNumber("low shot rpm", 0);

    private SmartDashboardNumber spikeThreshold = new SmartDashboardNumber("hood/hood-spike-threshold", 0.1);
    private SmartDashboardNumber normalizeSpeed = new SmartDashboardNumber("hood/hood-normalize-speed", 0.1);

    private SmartDashboardNumber pidTolerance = new SmartDashboardNumber("hood/hood-pid-tolerance", 0.1);
    private SmartDashboardNumber positionTolerance = new SmartDashboardNumber("hood/hood-position-tolerance", 0.1);

    private double nonClampedTargetRevolution;

    //for smartdashboard only
    private double targetHoodAngle;
    private double targetRPM;

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
                .withPeakReverseDutyCycle(1d)
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

        this.m_shooterMotor.getConfigurator().apply(shooterSlot0Configs);
        this.m_shooterMotor.getConfigurator().apply(shooterMotionMagicConfigs);
        this.m_hoodMotor.getConfigurator().apply(hoodSlot0Configs);
    }

    public void setHoodAngle(double angle) {
        this.m_hoodMotor.setControl(new PositionVoltage(MathUtil.clamp(this.angleToRotation(angle), kMinHoodRotation, kMaxHoodRotation))
                                        .withSlot(0)
                                        .withEnableFOC(true)
                                        .withOverrideBrakeDurNeutral(true)
        );
        this.targetHoodAngle = angle;
        this.nonClampedTargetRevolution = this.angleToRotation(angle);
    }

    public void setShooterRPM(double rpm) {
        if (Double.compare(rpm, 0d) == 0) {
            this.m_shooterMotor.setControl(new DutyCycleOut(0d).withOverrideBrakeDurNeutral(false));
        } else {
            this.m_shooterMotor.setControl(new MotionMagicVelocityVoltage(rpm / 60d)
                                            .withSlot(0)
                                            .withEnableFOC(true)
                                            .withOverrideBrakeDurNeutral(true)
            );
        }
        this.targetRPM = rpm;
    }

    public void setFenderShotState() {
        this.setHoodAngle(fenderAngle.getNumber());
        this.setShooterRPM(fenderRPM.getNumber());
    }

    public void setLowShotState() {
        this.setHoodAngle(lowAngle.getNumber());
        this.setShooterRPM(lowRPM.getNumber());
    }

    public void resetHood() {
        this.m_hoodMotor.setControl(new CoastOut());
        this.m_hoodMotor.setPosition(0d);
    }

    public boolean inSpikeCurrent() {
        return this.m_hoodMotor.getTorqueCurrent().getValueAsDouble() > this.spikeThreshold.getNumber();
    }

    private void setNormalizeSpeed() {
        this.m_hoodMotor.setControl(new DutyCycleOut(this.normalizeSpeed.getNumber()));
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

        SmartDashboard.putNumber("hood/hood-position", this.m_hoodMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("hood/hood-target-angle", this.targetHoodAngle);
        SmartDashboard.putNumber("hood/hood-torque-current", this.m_hoodMotor.getTorqueCurrent().getValueAsDouble());
        SmartDashboard.putBoolean("hood/hood-at-spike", this.inSpikeCurrent());

        if (this.autoAimEnabled) {
            if (this.onBlue) this.aimToTargetBlue();
            else this.aimToTargetRed(); 
        }
    }

    private void aimToTargetRed() {
        ShotParameter p = InterpolatingTable.getRed(Localization.getDistanceToTargetRed());
        this.setHoodAngle(p.pivotAngleDeg);
        this.setShooterRPM(p.rpm);
    }

    private void aimToTargetBlue() {
        ShotParameter p = InterpolatingTable.getBlue(Localization.getDistanceToTargetBlue());
        this.setHoodAngle(p.pivotAngleDeg);
        this.setShooterRPM(p.rpm);
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
                this.enableAutoAim();
            }, 
            () -> this.inSpikeCurrent(), 
            this);
    }

    public static Shooter getInstance() {
        if (instance == null) instance = new Shooter();
        return instance;
    }
}
