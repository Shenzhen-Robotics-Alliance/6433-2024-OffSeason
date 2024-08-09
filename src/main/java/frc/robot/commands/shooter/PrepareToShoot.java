package frc.robot.commands.shooter;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.utils.MapleShooterOptimization;

import java.util.function.Supplier;

public class PrepareToShoot extends Command {
    private final Shooter shooter;
    private final MapleShooterOptimization shooterOptimization;
    private final Supplier<Translation2d> targetPositionSupplier;
    private final Supplier<Translation2d> robotPositionSupplier;

    public PrepareToShoot(Shooter shooter, MapleShooterOptimization shooterOptimization, Supplier<Translation2d> targetPositionSupplier, Supplier<Translation2d> robotPositionSupplier) {
        this.shooter = shooter;
        this.shooterOptimization = shooterOptimization;
        this.targetPositionSupplier = targetPositionSupplier;
        this.robotPositionSupplier = robotPositionSupplier;

        super.addRequirements(shooter);
    }

    private MapleShooterOptimization.ShooterState initialState;
    @Override
    public void initialize() {
        initialState = shooterOptimization.getOptimizedShootingState(
                targetPositionSupplier.get(),
                robotPositionSupplier.get(),
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
