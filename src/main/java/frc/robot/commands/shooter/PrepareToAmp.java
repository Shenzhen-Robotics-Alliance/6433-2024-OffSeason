package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

public class PrepareToAmp extends SequentialCommandGroup {
    public PrepareToAmp(Shooter shooter, Intake intake) {
        addRequirements(intake, shooter);

        addCommands(Commands.run(shooter::runPrepareAmp, shooter)
                .beforeStarting(shooter::runPrepareAmp, shooter)
                .alongWith(Commands.run(intake::runIdle, intake))
        );
    }
}
