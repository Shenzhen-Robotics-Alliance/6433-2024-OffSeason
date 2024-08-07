package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;

public class IntakeIOReal implements IntakeIO {
    private final TalonSRX intakeMotor = new TalonSRX(9);
    private final DigitalInput beamBreak = new DigitalInput(3);
    @Override
    public void updateInputs(IntakeInputs inputs) {
        inputs.beamBreakTriggered = beamBreak.get();
    }

    @Override
    public void runIntakeVoltage(double volts) {
        intakeMotor.set(ControlMode.PercentOutput,
                volts / RobotController.getBatteryVoltage());
    }
}
