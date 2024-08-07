package frc.robot.commands.shooter;


import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

public class GrabNoteManual extends SequentialCommandGroup {
    private final Shooter shooter;
    private final Intake intake;

    public GrabNoteManual(Shooter shooter, Intake intake) {
        this.shooter = shooter;
        this.intake = intake;

        super.addRequirements(shooter, intake);

        super.addCommands(
                Commands.run(shooter::runIdle)
                        .until(shooter::isPitchInPosition)
        );

        super.addCommands(intake.runIntakeUntilNoteDetected());
    }
}
