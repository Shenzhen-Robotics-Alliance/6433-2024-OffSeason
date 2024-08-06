package frc.robot.tests;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotContainer;

import java.util.function.Supplier;

public class TestBuilder {
    public static SendableChooser<Supplier<Command>> buildTestsChooser(RobotContainer robotContainer) {
        final SendableChooser<Supplier<Command>> testsChooser = new SendableChooser<>();
        testsChooser.setDefaultOption("None", Commands::none);
        testsChooser.addOption("Wheels Calibration", WheelsCalibrationCTRE::new);
        testsChooser.addOption("Field Display Test", FieldDisplayTest::new);
        testsChooser.addOption("Robot Simulation Test", PhysicsSimulationTest::new);

        testsChooser.addOption(
                "Fly Wheels SysId Quasi-static (forward)",
                () -> robotContainer.shooter.flyWheelsSysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward)
        );
        testsChooser.addOption(
                "Fly Wheels SysId Quasi-static (reverse)",
                () -> robotContainer.shooter.flyWheelsSysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse)
        );
        testsChooser.addOption(
                "Fly Wheels SysId Dynamic (forward)",
                () -> robotContainer.shooter.flyWheelsSysIdRoutine.dynamic(SysIdRoutine.Direction.kForward)
        );
        testsChooser.addOption(
                "Fly Wheels SysId Dynamic (reverse)",
                () -> robotContainer.shooter.flyWheelsSysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse)
        );
        return testsChooser;
    }
}
