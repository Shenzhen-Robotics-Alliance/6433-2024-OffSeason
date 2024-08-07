package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot;
import frc.robot.subsystems.MapleSubsystem;
import frc.robot.utils.MechanismControl.MaplePIDController;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.FlyWheelsConfigs.*;
import static frc.robot.Constants.PitchConfigs.*;

public class Shooter extends MapleSubsystem {
    private final FlyWheelsIO flyWheelsIO;
    private final FlyWheelsInputsAutoLogged flyWheelsInputs;
    public final SysIdRoutine flyWheelsSysIdRoutine;

    private final SimpleMotorFeedforward flyWheelsFeedForward;
    /* velocity RPM of the shooter is the position of the trapezoid profile */
    private final TrapezoidProfile flyWheelsSpeedRPMProfile;
    private final PIDController flyWheelsFeedBack;
    private TrapezoidProfile.State flyWheelCurrentState;
    private double flyWheelsSetpointRPM;

    private final PitchIO pitchIO;
    private final PitchInputsAutoLogged pitchInputs;
    private final ArmFeedforward pitchFeedForward;
    private final PIDController pitchFeedBack;
    private final TrapezoidProfile pitchProfile;
    private TrapezoidProfile.State pitchCurrentState;
    private double pitchSetpointRad;

    public Shooter(FlyWheelsIO flyWheelsIO, PitchIO pitchIO) {
        super("Shooter");

        this.flyWheelsIO = flyWheelsIO;
        this.flyWheelsInputs = new FlyWheelsInputsAutoLogged();
        this.flyWheelsFeedForward = new SimpleMotorFeedforward(FLYWHEELS_KS, FLYWHEELS_KV, FLYWHEELS_KA);
        this.flyWheelsSpeedRPMProfile = new TrapezoidProfile(FLYWHEELS_RPM_PROFILE_CONSTRAIN);
        this.flyWheelsFeedBack = new MaplePIDController(FLYWHEELS_PID);
        this.flyWheelsSysIdRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null, null, null,
                        (state) -> Logger.recordOutput("Shooter/FlyWheelsSysIdState", state.toString())
                ),
                new SysIdRoutine.Mechanism(
                        voltageMeasure -> flyWheelsIO.runFlyWheelsVoltage(voltageMeasure.in(Volts)),
                        null,
                        this
                )
        );

        this.pitchIO = pitchIO;
        this.pitchInputs = new PitchInputsAutoLogged();
        this.pitchFeedForward = new ArmFeedforward(PITCH_KS, PITCH_KG, PITCH_KV, PITCH_KA);
        this.pitchFeedBack = new MaplePIDController(PITCH_PID);
        this.pitchProfile = new TrapezoidProfile(PITCH_PROFILE_CONSTRAIN);

        super.setDefaultCommand(Commands.run(this::runIdle, this));
    }

    @Override
    public void onReset() {
        this.flyWheelCurrentState = new TrapezoidProfile.State(0, 0);
        this.flyWheelsSetpointRPM = 0;
        this.pitchCurrentState = new TrapezoidProfile.State(PITCH_LOWEST_ROTATION_RAD, 0);
        this.pitchSetpointRad = PITCH_LOWEST_ROTATION_RAD;
    }

    @Override
    public void onEnable() {
        this.pitchIO.setPitchLock(true);
    }

    @Override
    public void onDisable() {
        this.pitchIO.setPitchLock(false);
    }

    @Override
    public void periodic(double dt, boolean enabled) {
        flyWheelsIO.updateInputs(flyWheelsInputs);
        Logger.processInputs("Shooter/FlyWheels", flyWheelsInputs);

        pitchIO.updateInputs(pitchInputs);
        Logger.processInputs("Shooter/Pitch", pitchInputs);

        if (!pitchInputs.calibrated)
            DriverStation.reportWarning("Warning!!! Pitch not calibrated!", false);
    }

    public void runIdle() {
        runShooterState(PITCH_LOWEST_ROTATION_RAD, 0);
    }

    public void runShooterState(double pitchAngleSetpointRadians, double shooterSetpointRPM) {
        runPitchCloseLoop(pitchAngleSetpointRadians);
        runFlyWheelCloseLoop(shooterSetpointRPM);

        Logger.recordOutput("Shooter/pitchAngleSetpointRadians", pitchAngleSetpointRadians);
        Logger.recordOutput("Shooter/shooterSetpointRPM", shooterSetpointRPM);
    }

    double previousStateVelocity = 0;
    private void runPitchCloseLoop(double pitchAngleSetpointRadians) {
        if (pitchAngleSetpointRadians < PITCH_LOWEST_ROTATION_RAD || pitchAngleSetpointRadians > PITCH_HIGHER_LIMIT_RAD || !pitchInputs.calibrated) {
            if (pitchInputs.calibrated)
                DriverStation.reportWarning("attempting to set a set-point out of range: " + pitchAngleSetpointRadians, false);
            pitchIO.runPitchVoltage(0);
            return;
        }

        this.pitchCurrentState = pitchProfile.calculate(
                Robot.defaultPeriodSecs,
                pitchCurrentState,
                new TrapezoidProfile.State(pitchAngleSetpointRadians, 0)
        );

        Logger.recordOutput("pitchProfileGoalPosition (deg)", Math.toDegrees(pitchCurrentState.position));
        final double pitchAcceleration = (pitchCurrentState.velocity - previousStateVelocity) / Robot.defaultPeriodSecs;
        previousStateVelocity = pitchCurrentState.velocity;
        final double
                feedForwardVoltage = pitchFeedForward.calculate(
                        pitchCurrentState.position, pitchCurrentState.velocity, pitchAcceleration
                ),
                feedBackVoltage = pitchFeedBack.calculate(
                        pitchInputs.pitchAngleRad, pitchCurrentState.position
                );

        final double
                safetyConstrainLow = pitchInputs.pitchAngleRad <= PITCH_LOWEST_ROTATION_RAD ? 0 : -10,
                safetyConstrainHigh = pitchInputs.pitchAngleRad >= PITCH_HIGHER_LIMIT_RAD ? 0 : 12,
                pitchVoltage = MathUtil.clamp(
                        feedForwardVoltage + feedBackVoltage, safetyConstrainLow, safetyConstrainHigh
                );

        Logger.recordOutput("Pitch Actual Position (Deg)", Math.toDegrees(getPitchAngleRad()));
        Logger.recordOutput("Pitch Correction Voltage", pitchVoltage);
        pitchIO.runPitchVoltage(pitchVoltage);
    }

    private void runFlyWheelCloseLoop(double shooterSetpointRPM) {
        if (getPitchAngleRad() < Math.toRadians(12) && shooterSetpointRPM!=0) {
            DriverStation.reportWarning("Pitch too low, not starting shooter for now...", false);
            shooterSetpointRPM = 0;
        }
        if (shooterSetpointRPM == 0) {
            flyWheelsIO.runFlyWheelsVoltage(0);
            return;
        }

        flyWheelCurrentState = flyWheelsSpeedRPMProfile.calculate(
                Robot.defaultPeriodSecs,
                flyWheelCurrentState,
                new TrapezoidProfile.State(shooterSetpointRPM, 0)
        );
        Logger.recordOutput("shooterProfileGoalRPM", flyWheelCurrentState.position);

        final double feedForwardVoltage = flyWheelsFeedForward.calculate(flyWheelCurrentState.position, flyWheelCurrentState.velocity),
                feedBackVoltage = flyWheelsFeedBack.calculate(getFlyWheelRPM(), flyWheelCurrentState.position);
        flyWheelsIO.runFlyWheelsVoltage(feedForwardVoltage + feedBackVoltage);
    }

    @AutoLogOutput(key = "Shooter/ActualShooterSpeedRPM")
    public double getFlyWheelRPM() {
        return flyWheelsInputs.flyWheelsEncoderVelocityRevPerSec * 60.0;
    }

    public boolean isFlyWheelReady() {
        return Math.abs(getFlyWheelRPM() - flyWheelsSetpointRPM) < FLYWHEELS_PID.errorTolerance;
    }

    public double getPitchAngleRad() {
        return pitchInputs.pitchAngleRad;
    }

    public boolean isPitchInPosition() {
        return Math.abs(getPitchAngleRad() - pitchSetpointRad) < PITCH_PID.errorTolerance;
    }
}
