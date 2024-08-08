package frc.robot.commands.shooter;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.HolonomicDriveSubsystem;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.utils.MapleShooterOptimization;

import java.util.function.Supplier;

public class AimAtSpeaker extends Command {
    private final Shooter shooter;
    private final MapleShooterOptimization shooterOptimization;
    private final HolonomicDriveSubsystem drive;
    private final Supplier<Translation2d> targetPositionSupplier;

    public AimAtSpeaker(Shooter shooter, MapleShooterOptimization shooterOptimization, HolonomicDriveSubsystem drive, Supplier<Translation2d> targetPositionSupplier) {
        this.shooter = shooter;
        this.shooterOptimization = shooterOptimization;
        this.drive = drive;
        this.targetPositionSupplier = targetPositionSupplier;

        super.addRequirements(shooter);
    }

    @Override
    public void initialize() {
        /* update the shooter set-point so command does not execute early */
        execute();
    }

    @Override
    public void execute() {
        shooter.runShooterAimingState(shooterOptimization.getOptimizedShootingState(
                targetPositionSupplier.get(),
                drive.getPose().getTranslation(),
                drive.getMeasuredChassisSpeedsFieldRelative()
        ));
    }
}
