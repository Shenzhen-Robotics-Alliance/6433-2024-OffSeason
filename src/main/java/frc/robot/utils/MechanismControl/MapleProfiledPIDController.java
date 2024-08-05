package frc.robot.utils.MechanismControl;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class MapleProfiledPIDController extends ProfiledPIDController {
    private final MaplePIDController.MaplePIDConfig pidConfig;
    public MapleProfiledPIDController(MaplePIDController.MaplePIDConfig pidConfig, TrapezoidProfile.Constraints constraints) {
        super(pidConfig.Kp, pidConfig.Ki, pidConfig.Kd, constraints);
        this.pidConfig = pidConfig;
    }

    @Override
    public double calculate(double measurement) {
        return MathUtil.clamp(super.calculate(measurement), -pidConfig.maximumPower, pidConfig.maximumPower);
    }
}
