package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;

import static frc.robot.Constants.PitchConfigs.*;

public class PitchIOReal implements PitchIO {
    private final TalonFX pitchFalcon = new TalonFX(15);
    private final StatusSignal<Double> pitchMotorSuppliedAmps, pitchRelativeEncoderPositionRev, pitchEncoderVelocityRevPerSec;
    private final DigitalInput limitSwitch = new DigitalInput(0);

    private double encoderPositionAtLowestPoint;
    public PitchIOReal() {
        pitchMotorSuppliedAmps = pitchFalcon.getSupplyCurrent();
        pitchRelativeEncoderPositionRev = pitchFalcon.getPosition();
        pitchEncoderVelocityRevPerSec = pitchFalcon.getVelocity();
        encoderPositionAtLowestPoint = pitchRelativeEncoderPositionRev.getValue();
        BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                pitchMotorSuppliedAmps, pitchRelativeEncoderPositionRev, pitchEncoderVelocityRevPerSec
        );
    }
    @Override
    public void updateInputs(PitchInputs pitchInputs) {
        BaseStatusSignal.refreshAll(
                pitchMotorSuppliedAmps, pitchRelativeEncoderPositionRev, pitchEncoderVelocityRevPerSec
        );

        pitchInputs.calibrated |= limitSwitch.get();
        if (limitSwitch.get())
            encoderPositionAtLowestPoint = pitchRelativeEncoderPositionRev.getValue();

        pitchInputs.pitchSuppliedCurrentAmps = pitchMotorSuppliedAmps.getValue();
        pitchInputs.pitchAngularVelocityRadPerSec = Units.rotationsToRadians(
                pitchEncoderVelocityRevPerSec.getValue() / GEAR_RATIO
        );
        pitchInputs.pitchAngleRad = Units.rotationsToRadians(
                (pitchRelativeEncoderPositionRev.getValue() - encoderPositionAtLowestPoint)
                        / GEAR_RATIO
        ) + PITCH_LOWEST_ROTATION_RAD;
    }

    @Override
    public void runPitchVoltage(double volts) {
        final VoltageOut voltageOut = new VoltageOut(volts).withEnableFOC(true);
        pitchFalcon.setControl(voltageOut);
    }

    @Override
    public void setPitchLock(boolean enabled) {
        pitchFalcon.setNeutralMode(enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }
}
