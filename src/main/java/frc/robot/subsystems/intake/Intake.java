package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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

        setDefaultCommand(Commands.run(this::runIdle, this));
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
        final SequentialCommandGroup commandGroup = new SequentialCommandGroup();
        commandGroup.addRequirements(this);
        commandGroup.addCommands(Commands.run(() -> io.runIntakeVoltage(12), this)
                .until(() -> inputs.beamBreakTriggered));
        commandGroup.addCommands(Commands.run(() -> io.runIntakeVoltage(-3), this)
                .withTimeout(0.4));
        return commandGroup;
    }

    public Command shootNoteUntilNoteGone() {
        return Commands.run(() -> io.runIntakeVoltage(12), this)
                .until(() -> !inputs.beamBreakTriggered);
    }

    public void runIdle() {
        io.runIntakeVoltage(0);
    }
}
