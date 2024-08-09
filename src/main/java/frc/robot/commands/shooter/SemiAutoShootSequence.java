package frc.robot.commands.shooter;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.drive.HolonomicDriveSubsystem;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.utils.MapleShooterOptimization;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class SemiAutoShootSequence extends ShootAtPositionSequence {
    public SemiAutoShootSequence(Shooter shooter, Intake intake, MapleShooterOptimization shooterOptimization, HolonomicDriveSubsystem drive, Supplier<Translation2d> targetPositionSupplier, BooleanSupplier chassisRotationInPositionSupplier) {
        super(
                shooter, intake, shooterOptimization, drive, targetPositionSupplier,
                () -> drive.getPose().getTranslation(),
                chassisRotationInPositionSupplier
        );
    }
}
