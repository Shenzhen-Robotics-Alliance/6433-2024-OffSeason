package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import org.littletonrobotics.junction.Logger;

public class FlyWheelsIOReal implements FlyWheelsIO {
    private final TalonFX shooterFalcon1, shooterFalcon2;
    private final StatusSignal<Double>
            shooter1SuppliedCurrentAmps, shooter2SuppliedCurrentAmps, shooterPositionRev, shooterVelocityRevPerSec;
    private final DigitalInput limitSwitch;
    public FlyWheelsIOReal() {
        this.shooterFalcon1 = new TalonFX(12);
        this.shooterFalcon2 = new TalonFX(13);
        this.shooterFalcon2.setInverted(true);
        this.limitSwitch = new DigitalInput(0);

        this.shooter1SuppliedCurrentAmps = shooterFalcon1.getSupplyCurrent();
        this.shooter2SuppliedCurrentAmps = shooterFalcon2.getSupplyCurrent();
        this.shooterPositionRev = shooterFalcon1.getPosition();
        this.shooterVelocityRevPerSec = shooterFalcon1.getVelocity();

        BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                shooter1SuppliedCurrentAmps, shooter2SuppliedCurrentAmps, shooterPositionRev, shooterVelocityRevPerSec
        );
        this.shooterFalcon1.optimizeBusUtilization();
        this.shooterFalcon2.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(FlyWheelsIO.FlyWheelsInputs inputs) {
        BaseStatusSignal.refreshAll(
                shooter1SuppliedCurrentAmps, shooter2SuppliedCurrentAmps, shooterPositionRev, shooterVelocityRevPerSec
        );
        inputs.flyWheelsSuppliedCurrentAmps = this.shooter1SuppliedCurrentAmps.getValue() + this.shooter2SuppliedCurrentAmps.getValue();
        inputs.flyWheelsEncoderVelocityRevPerSec = this.shooterVelocityRevPerSec.getValue();
        inputs.flyWheelEncoderPositionRev = this.shooterPositionRev.getValue();
    }

    @Override
    public void runFlyWheelsVoltage(double volts) {
        final VoltageOut control = new VoltageOut(volts).withEnableFOC(false);
        Logger.recordOutput("Shooter/flyWheelsAppliedVolts", volts);
        shooterFalcon1.setControl(control);
        shooterFalcon2.setControl(control);
    }
}
