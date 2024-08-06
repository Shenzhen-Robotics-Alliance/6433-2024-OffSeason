package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot;
import frc.robot.subsystems.MapleSubsystem;
import frc.robot.utils.MechanismControl.MaplePIDController;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.ShooterConfigs.*;

public class Shooter extends MapleSubsystem {
    private final FlyWheelsIO flyWheelsIO;
    private final FlyWheelsIO.FlyWheelsInputs flyWheelsInputs;
    public final SysIdRoutine flyWheelsSysIdRoutine;

    private final SimpleMotorFeedforward flyWheelsFeedForward = new SimpleMotorFeedforward(FLYWHEELS_KS, FLYWHEELS_KV, FLYWHEELS_KA);
    /* velocity RPM of the shooter is the position of the trapezoid profile */
    private final TrapezoidProfile flyWheelsSpeedRPMProfile = new TrapezoidProfile(FLYWHEELS_RPM_CONSTRAIN);
    private final PIDController flyWheelsFeedBack = new MaplePIDController(FLYWHEELS_PID);
    private TrapezoidProfile.State flyWheelCurrentState;
    public Shooter(FlyWheelsIO flyWheelsIO) {
        super("Shooter");
        this.flyWheelsIO = flyWheelsIO;
        this.flyWheelsInputs = new FlyWheelsIO.FlyWheelsInputs();

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
    }

    @Override
    public void onReset() {
        flyWheelCurrentState = new TrapezoidProfile.State(0, 0);
    }

    @Override
    public void onEnable() {

    }

    @Override
    public void onDisable() {

    }

    @Override
    public void periodic(double dt, boolean enabled) {
        flyWheelsIO.updateInputs(flyWheelsInputs);
        // Logger.processInputs("Shooter", inputs);
    }

    public void runShooterState(double pitchAngleSetpointRadians, double shooterSetpointRPM) {
        runArmCloseLoop(pitchAngleSetpointRadians);
        runFlyWheelCloseLoop(shooterSetpointRPM);

        Logger.recordOutput("Shooter/pitchAngleSetpointRadians", pitchAngleSetpointRadians);
        Logger.recordOutput("Shooter/shooterSetpointRPM", shooterSetpointRPM);
    }

    private void runArmCloseLoop(double pitchAngleSetpointRadians) {

    }

    private void runFlyWheelCloseLoop(double shooterSetpointRPM) {
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
        return flyWheelsFeedBack.atSetpoint();
    }
}
