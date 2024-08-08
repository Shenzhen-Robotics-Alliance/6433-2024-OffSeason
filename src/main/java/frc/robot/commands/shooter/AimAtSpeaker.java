package frc.robot.commands.shooter;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.HolonomicDriveSubsystem;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.utils.MapleShooterOptimization;

import java.util.function.Supplier;

public class AimAtSpeaker extends SequentialCommandGroup {
    public AimAtSpeaker(Shooter shooter, MapleShooterOptimization shooterOptimization, HolonomicDriveSubsystem drive, Supplier<Translation2d> targetPositionSupplier) {
        super.addCommands(new PrepareToShoot(shooter, shooterOptimization, targetPositionSupplier, drive));
        super.addCommands(new MaintainAimAtSpeaker(shooter, shooterOptimization, drive, targetPositionSupplier));
    }
}
