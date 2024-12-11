package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils3006.SmartDashboardNumber;
import frc.robot.vision.Localization;

public class Turret extends SubsystemBase{
    private static Turret instance = null;

    //TODO: Most clockwise is min, most counterclockwise is max, aka CCW+
    //TODO: 0 angle @ centered to robot pointing robot releative front
    public static final double kMaxTurretRotation = 0;
    public static final Rotation2d kMaxTurretAngle = new Rotation2d();
    public static final double kMinTurretRotation = 0;
    public static final Rotation2d kMinTurretAngle = new Rotation2d();

    private TalonFX m_turretMotor = new TalonFX(40);

    private Slot0Configs slot0Configs = new Slot0Configs();
    private MotionMagicConfigs motionConfigs = new MotionMagicConfigs();

    private SmartDashboardNumber kTurretAccel = new SmartDashboardNumber("turret/turret-motion-accel", 0);
    private SmartDashboardNumber kTurretVel = new SmartDashboardNumber("turret/turret-motion-velocity", 0);

    private SmartDashboardNumber turretKs = new SmartDashboardNumber("turret/ks", 0);
    private SmartDashboardNumber turretKa = new SmartDashboardNumber("turret/ka", 0);
    private SmartDashboardNumber turretKv = new SmartDashboardNumber("turret/kv", 0); //to be tuned;
    private SmartDashboardNumber turretKp = new SmartDashboardNumber("turret/kp", 0); //to be tuned;
    private SmartDashboardNumber turretKi = new SmartDashboardNumber("turret/ki", 0);
    private SmartDashboardNumber turretKd = new SmartDashboardNumber("turret/kd", 0);

    private SmartDashboardNumber spikeThreshold = new SmartDashboardNumber("turret/turret-spike-threshold", 0);
    private SmartDashboardNumber normalizeSpeed = new SmartDashboardNumber("turret/normalize-reset-speed", 0.1);

    private SmartDashboardNumber pidTolerance = new SmartDashboardNumber("turret/turret-pid-Tolerance", 0.1);
    private SmartDashboardNumber positionTolerance = new SmartDashboardNumber("turret/turret-position-tolerance", 0.1);

    private boolean autoAimEnabled = false;
    private DriverStation.Alliance alliance;

    private double nonClampedTargetRevolution;

    private Turret() {
        super("turret");

        this.alliance = (DriverStation.getAlliance().isPresent()) ? DriverStation.getAlliance().get() : DriverStation.Alliance.Blue;

        this.m_turretMotor.getConfigurator().apply(
            new MotorOutputConfigs()
                .withInverted(InvertedValue.CounterClockwise_Positive)
                .withPeakForwardDutyCycle(1d)
                .withPeakReverseDutyCycle(-1d)
                .withNeutralMode(NeutralModeValue.Brake)
        );

        this.slot0Configs = new Slot0Configs()
            .withKS(turretKs.getNumber())
            .withKA(turretKa.getNumber())
            .withKV(turretKv.getNumber())
            .withKP(turretKp.getNumber())
            .withKI(turretKi.getNumber())
            .withKD(turretKd.getNumber());

        this.motionConfigs = new MotionMagicConfigs()
            .withMotionMagicAcceleration(kTurretAccel.getNumber())
            .withMotionMagicCruiseVelocity(kTurretVel.getNumber());

        this.m_turretMotor.getConfigurator().apply(slot0Configs);
        this.m_turretMotor.getConfigurator().apply(motionConfigs);
    }

    public void setTurretPosition(Rotation2d angle) {
        this.m_turretMotor.setControl(
            new MotionMagicVoltage(MathUtil.clamp(this.angleToRotation(angle), kMinTurretRotation, kMaxTurretRotation))
                .withSlot(0)
                .withEnableFOC(true)
                .withOverrideBrakeDurNeutral(true)
        );
        this.nonClampedTargetRevolution = this.angleToRotation(angle);
    }

    public void reset() {
        this.m_turretMotor.setControl(new CoastOut());
        this.m_turretMotor.setPosition(0d);
        this.m_turretMotor.setControl(new DutyCycleOut(0));
    }

    public void enableAutoAim() {
        this.autoAimEnabled = true;
    }

    public void disableAutoAim() {
        this.autoAimEnabled = false;
    }

    private void setNormalizeSpeed() {
        this.m_turretMotor.setControl(
            new DutyCycleOut(this.normalizeSpeed.getNumber())
        );
    }

    public boolean inSpikeCurrent() {
        return Math.abs(this.m_turretMotor.getTorqueCurrent().getValueAsDouble()) > this.spikeThreshold.getNumber();
    }

    public boolean isReady() {
        return Math.abs(this.m_turretMotor.getClosedLoopError().getValueAsDouble()) < this.pidTolerance.getNumber() && 
            Math.abs(this.m_turretMotor.getClosedLoopReference().getValueAsDouble() - this.nonClampedTargetRevolution) < this.positionTolerance.getNumber(); 
    }

    private double angleToRotation(Rotation2d angle) {
        return ((kMaxTurretRotation - kMinTurretRotation) / (kMaxTurretAngle.getDegrees() - kMinTurretAngle.getDegrees())) * (angle.getDegrees() - kMinTurretAngle.getDegrees()) + kMinTurretRotation;
    }

    @Override
    public void periodic() {
        if (turretKs.hasChanged()
        || turretKv.hasChanged()
        || turretKp.hasChanged()
        || turretKi.hasChanged()
        || turretKd.hasChanged()
        || turretKa.hasChanged()) {
            slot0Configs.kS = turretKs.getNumber();
            slot0Configs.kV = turretKv.getNumber();
            slot0Configs.kP = turretKp.getNumber();
            slot0Configs.kI = turretKi.getNumber();
            slot0Configs.kD = turretKd.getNumber();
            slot0Configs.kA = turretKa.getNumber();

            if (!Utils.isSimulation()) this.m_turretMotor.getConfigurator().apply(slot0Configs);
            System.out.println("applyied");
        }

        if (kTurretAccel.hasChanged() || kTurretVel.hasChanged()) {
            this.motionConfigs.MotionMagicAcceleration = kTurretAccel.getNumber();
            this.motionConfigs.MotionMagicCruiseVelocity = kTurretVel.getNumber();
            this.m_turretMotor.getConfigurator().apply(this.motionConfigs);
        }

        SmartDashboard.putNumber("turret/turret-acceleration", this.m_turretMotor.getAcceleration().getValueAsDouble());
        SmartDashboard.putNumber("turret/turret-velocity", this.m_turretMotor.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("turret/turret-position", this.m_turretMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("turret/turret-motor-torque-current", this.m_turretMotor.getTorqueCurrent().getValueAsDouble());
        SmartDashboard.putNumber("turret/turret-closed-loop-error", this.m_turretMotor.getClosedLoopError().getValueAsDouble());

        SmartDashboard.putBoolean("turret/turret-auto-aim-enabled", this.autoAimEnabled);
        SmartDashboard.putBoolean("turret/turret-at-spike", this.inSpikeCurrent());

        if (this.autoAimEnabled) {
            if (this.alliance == DriverStation.Alliance.Blue) aimToTargetBlue();
            else aimToTargetRed();
        }
    }

    private void aimToTargetRed() {
        this.setTurretPosition(
            Localization.getAngleToRed().minus(
                Localization.getPose2d().getRotation()
            )
        );
    }

    private void aimToTargetBlue() {
        this.setTurretPosition(
            Localization.getAngleToBlue().minus(
                Localization.getPose2d().getRotation()
            )
        );
    }

    public Command normalizeTurretCommand() {
        return new FunctionalCommand(
            () -> {this.disableAutoAim();
                this.setNormalizeSpeed();
            }, 
            () -> {}, 
            (interrupted) -> {this.reset();}, 
            () -> this.inSpikeCurrent(), 
            this);
    }

    public static Turret getInstance() {
        if (instance == null) instance = new Turret();
        return instance;
    }
}
