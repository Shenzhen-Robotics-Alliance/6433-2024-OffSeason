package frc.robot.commands.shooter;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.HolonomicDriveSubsystem;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.utils.MapleShooterOptimization;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class ShootNoteAutoAim extends SequentialCommandGroup {
    public ShootNoteAutoAim(Shooter shooter, Intake intake, MapleShooterOptimization shooterOptimization, HolonomicDriveSubsystem drive, Supplier<Translation2d> targetPositionSupplier) {
        super.addRequirements(shooter);

        final BooleanSupplier readyToShoot = () -> shooter.isReady()
                && shooterOptimization.isTargetInRange(targetPositionSupplier.get(), drive.getPose().getTranslation());
        final Command shooterAimAtSpeaker = new MaintainAimAtSpeaker(shooter, shooterOptimization, drive, targetPositionSupplier),
                shoot = intake.shootNoteUntilNoteGone(),
                shootWhenAimComplete = Commands.waitUntil(readyToShoot).andThen(shoot);

        super.addCommands(shooterAimAtSpeaker.alongWith(shootWhenAimComplete));
    }
}
