package frc.robot.commands.shooter;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.HolonomicDriveSubsystem;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.utils.MapleShooterOptimization;

import java.util.function.Supplier;

public class PrepareToShoot extends Command {
    private final Shooter shooter;
    private final MapleShooterOptimization shooterOptimization;
    private final Supplier<Translation2d> targetPositionSupplier;
    private final HolonomicDriveSubsystem drive;

    public PrepareToShoot(Shooter shooter, MapleShooterOptimization shooterOptimization, Supplier<Translation2d> targetPositionSupplier, HolonomicDriveSubsystem drive) {
        this.shooter = shooter;
        this.shooterOptimization = shooterOptimization;
        this.targetPositionSupplier = targetPositionSupplier;
        this.drive = drive;

        super.addRequirements(shooter);
    }

    private MapleShooterOptimization.ShooterState initialState;
    @Override
    public void initialize() {
        initialState = shooterOptimization.getOptimizedShootingState(
                targetPositionSupplier.get(),
                drive.getPose().getTranslation(),
                new ChassisSpeeds()
        );
    }
    @Override
    public void execute() {
        shooter.runProfiledShooterState(initialState.shooterAngleDegrees, initialState.shooterRPM);
    }

    @Override
    public boolean isFinished() {
        return shooter.isReady();
    }
}
