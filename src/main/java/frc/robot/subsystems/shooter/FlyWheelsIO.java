package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface FlyWheelsIO {
    @AutoLog
    class FlyWheelsInputs {
        double flyWheelsSuppliedCurrentAmps = 0.0;
        double flyWheelEncoderPositionRev = 0.0;
        double flyWheelsEncoderVelocityRevPerSec = 0.0;
    }

    void updateInputs(FlyWheelsInputs inputs);

    default void runFlyWheelsVoltage(double volts) {}
}
