package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    class IntakeInputs {
        boolean beamBreakTriggered = true;
    }

    void updateInputs(IntakeInputs inputs);

    default void runIntakeVoltage(double volts) {}
}
