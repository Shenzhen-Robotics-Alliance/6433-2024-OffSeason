package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot;
import frc.robot.subsystems.MapleSubsystem;
import frc.robot.utils.Alert;
import frc.robot.utils.MapleShooterOptimization;
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
    private double flyWheelsSetpointRPM;

    private final PitchIO pitchIO;
    private final PitchInputsAutoLogged pitchInputs;
    private final ArmFeedforward pitchFeedForward;
    private final PIDController pitchFeedBack;
    private final TrapezoidProfile pitchProfile;
    private double pitchSetpointRad;

    private final Alert pitchNotCalibratedAlert = new Alert("Pitch not calibrated!", Alert.AlertType.ERROR);
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

        pitchNotCalibratedAlert.setActivated(false);
    }

    @Override
    public void onReset() {
        this.flyWheelsCurrentState = new TrapezoidProfile.State(0, 0);
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

        pitchNotCalibratedAlert.setActivated(!pitchInputs.calibrated);
    }

    private void runFlyWheelsControlLoops() {
        final double feedForwardVoltage = flyWheelsFeedForward.calculate(flyWheelsCurrentState.position,  flyWheelsCurrentState.velocity),
                feedBackVoltage = flyWheelsFeedBack.calculate(getFlyWheelRPM(), flyWheelsCurrentState.position);
        flyWheelsIO.runFlyWheelsVoltage(feedForwardVoltage + feedBackVoltage);
    }

    private TrapezoidProfile.State flyWheelsCurrentState;
    private void runFlyWheelsProfile(double shooterSetpointRPM) {
        if (getPitchAngleRad() < Math.toRadians(12) && shooterSetpointRPM!=0) {
            DriverStation.reportWarning("Pitch too low, not starting shooter for now...", false);
            shooterSetpointRPM = 0;
        }

        flyWheelsCurrentState = flyWheelsSpeedRPMProfile.calculate(
                Robot.defaultPeriodSecs,
                flyWheelsCurrentState,
                new TrapezoidProfile.State(shooterSetpointRPM, 0)
        );
        Logger.recordOutput("shooterProfileGoalRPM", flyWheelsCurrentState.position);
    }

    double previousStateVelocity = 0;
    private void runPitchControlLoops() {
        if (pitchCurrentState.position < PITCH_LOWEST_ROTATION_RAD || pitchCurrentState.position > PITCH_HIGHER_LIMIT_RAD || !pitchInputs.calibrated) {
            if (pitchInputs.calibrated)
                DriverStation.reportWarning(
                        "attempting to run a pitch setpoint out of range: "
                                + pitchCurrentState.position
                        , true
                );
            pitchIO.runPitchVoltage(0);
            return;
        }
        final double pitchAcceleration = (pitchCurrentState.velocity - previousStateVelocity) / Robot.defaultPeriodSecs;
        previousStateVelocity = pitchCurrentState.velocity;
        final double
                feedForwardVoltage = pitchFeedForward.calculate(
                pitchInputs.pitchAngleRad, pitchCurrentState.velocity, pitchAcceleration
        ),
                feedBackVoltage = pitchFeedBack.calculate(
                        pitchInputs.pitchAngleRad, pitchCurrentState.position
                );

        final double
                safetyConstrainLow = pitchInputs.pitchAngleRad <= PITCH_LOWEST_ROTATION_RAD ? 0 : -10,
                safetyConstrainHigh = pitchInputs.pitchAngleRad > PITCH_HIGHER_LIMIT_RAD ? 0 : 12,
                pitchVoltage = MathUtil.clamp(
                        feedForwardVoltage + feedBackVoltage, safetyConstrainLow, safetyConstrainHigh
                );

        Logger.recordOutput("Shooter/Pitch Actual Position (Deg)", Math.toDegrees(getPitchAngleRad()));
        Logger.recordOutput("Shooter/Pitch Correction Voltage", pitchVoltage);
        pitchIO.runPitchVoltage(pitchVoltage);
    }

    private TrapezoidProfile.State pitchCurrentState;
    private void runPitchProfile(double pitchAngleSetpointRadians) {
        this.pitchCurrentState = pitchProfile.calculate(
                Robot.defaultPeriodSecs,
                pitchCurrentState,
                new TrapezoidProfile.State(pitchAngleSetpointRadians, 0)
        );

        Logger.recordOutput("Shooter/pitchProfileGoalPosition (deg)", Math.toDegrees(pitchCurrentState.position));
    }

    public void runProfiledShooterState(double pitchAngleSetpointRadians, double flyWheelsSetpointRPM) {
        this.pitchSetpointRad = pitchAngleSetpointRadians;
        this.flyWheelsSetpointRPM = flyWheelsSetpointRPM;

        runPitchProfile(pitchAngleSetpointRadians);
        runPitchControlLoops();

        runFlyWheelsProfile(flyWheelsSetpointRPM);
        runFlyWheelsControlLoops();

        Logger.recordOutput("Shooter/pitchAngleSetpointRadians", pitchAngleSetpointRadians);
        Logger.recordOutput("Shooter/shooterSetpointRPM", flyWheelsSetpointRPM);
    }

    public void runShooterAimingState(MapleShooterOptimization.ShooterState shootingState) {
        this.pitchCurrentState = new TrapezoidProfile.State(
                Math.toRadians(shootingState.shooterAngleDegrees),
                Math.toRadians(shootingState.shooterAngleChangeRateDegreesPerSecond)
        );
        this.previousStateVelocity = pitchCurrentState.velocity; // this disables acceleration FF
        this.pitchSetpointRad = pitchCurrentState.position;
        runPitchControlLoops();

        this.flyWheelsCurrentState = new TrapezoidProfile.State(
                shootingState.shooterRPM,
                shootingState.shooterRPMChangeRateRPMPerSeconds
        );
        this.flyWheelsSetpointRPM = shootingState.shooterRPM;
        runFlyWheelsControlLoops();
    }

    public void runIdle() {
        runProfiledShooterState(Math.toRadians(12), 0);
    }

    public void runPrepareAmp() {
        runProfiledShooterState(Math.toRadians(60), 500);
    }

    public void runAmp() {
        runProfiledShooterState(PITCH_HIGHER_LIMIT_RAD, 700);
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

    public boolean isReady() {
        return isFlyWheelReady() && isPitchInPosition();
    }
}
