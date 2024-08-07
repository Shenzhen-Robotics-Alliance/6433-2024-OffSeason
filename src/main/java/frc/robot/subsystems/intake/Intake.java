package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.subsystems.MapleSubsystem;
import frc.robot.utils.Alert;
import org.littletonrobotics.junction.Logger;

public class Intake extends MapleSubsystem {
    private final IntakeIO io;
    private final IntakeInputsAutoLogged inputs;

    private boolean beamBrakeAlwaysTrue = true;

    private final Alert beamBrakeAlwaysBlockedAlert = new Alert("Intake Beam Breaker Always Blocked", Alert.AlertType.WARNING);
    public Intake(IntakeIO io) {
        super("Intake");
        this.io = io;
        this.inputs = new IntakeInputsAutoLogged();
        beamBrakeAlwaysBlockedAlert.setActivated(false);
    }


    @Override
    public void onReset() {

    }

    @Override
    public void periodic(double dt, boolean enabled) {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
        beamBrakeAlwaysTrue &= inputs.beamBreakTriggered;
        this.beamBrakeAlwaysBlockedAlert.setActivated(beamBrakeAlwaysTrue);
    }

    public Command runIntakeUntilNoteDetected() {
        return new FunctionalCommand(
                () -> {},
                () -> io.runIntakeVoltage(12),
                interrupted ->io.runIntakeVoltage(0),
                () -> inputs.beamBreakTriggered,
                this
        );
    }

    public Command shootNoteUntilNoteGone() {
        return new FunctionalCommand(
                () -> {},
                () -> io.runIntakeVoltage(8),
                interrupted ->io.runIntakeVoltage(0),
                () -> !inputs.beamBreakTriggered,
                this
        );
    }
}
