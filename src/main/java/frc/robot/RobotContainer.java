// Original Source:
// https://github.com/Mechanical-Advantage/AdvantageKit/tree/main/example_projects/advanced_swerve_drive/src/main, Copyright 2021-2024 FRC 6328
// Modified by 5516 Iron Maple https://github.com/Shenzhen-Robotics-Alliance/

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.autos.Auto;
import frc.robot.autos.AutoBuilder;
import frc.robot.commands.drive.JoystickDrive;
import frc.robot.commands.drive.JoystickDriveAndAimAtTarget;
import frc.robot.commands.shooter.*;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.drive.IO.GyroIOPigeon2;
import frc.robot.subsystems.drive.IO.GyroIOSim;
import frc.robot.subsystems.drive.IO.ModuleIOSim;
import frc.robot.subsystems.drive.IO.ModuleIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOReal;
import frc.robot.subsystems.shooter.PitchIOReal;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.FlyWheelsIOReal;
import frc.robot.subsystems.vision.apriltags.AprilTagVision;
import frc.robot.subsystems.vision.apriltags.AprilTagVisionIOReal;
import frc.robot.subsystems.vision.apriltags.ApriltagVisionIOSim;
import frc.robot.tests.*;
import frc.robot.utils.CompetitionFieldUtils.Simulation.CompetitionFieldSimulation;
import frc.robot.utils.CompetitionFieldUtils.Simulation.Crescendo2024FieldSimulation;
import frc.robot.utils.CompetitionFieldUtils.Simulation.OpponentRobotSimulation;
import frc.robot.utils.CompetitionFieldUtils.Simulation.SwerveDriveSimulation;
import frc.robot.utils.Config.MapleConfigFile;
import frc.robot.utils.Config.PhotonCameraProperties;
import frc.robot.utils.MapleJoystickDriveInput;
import frc.robot.utils.MapleShooterOptimization;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import java.io.IOException;
import java.util.List;
import java.util.function.Supplier;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    private final PowerDistribution powerDistribution;
    // Subsystems
    public final SwerveDrive drive;
    public final AprilTagVision aprilTagVision;
    public final Shooter shooter;
    private final Intake intake;

    // Controller
    private final CommandXboxController driverController = new CommandXboxController(0),
            operatorController = new CommandXboxController(1);

    // Dashboard inputs
    public enum DriverMode {
        LEFT_HANDED,
        RIGHT_HANDED
    }
    private final LoggedDashboardChooser<DriverMode> driverModeChooser;
    private LoggedDashboardChooser<Auto> autoChooser;
    private final SendableChooser<Supplier<Command>> testChooser;

    // Simulation
    private CompetitionFieldSimulation fieldSimulation = null;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        final MapleConfigFile chassisCalibrationFile;
        try {
            chassisCalibrationFile = MapleConfigFile.fromDeployedConfig(
                    "ChassisWheelsCalibration",
                    Constants.ROBOT_NAME
            );
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
        final List<PhotonCameraProperties> camerasProperties = PhotonCameraProperties.loadCamerasPropertiesFromConfig(Constants.ROBOT_NAME);
        final MapleConfigFile.ConfigBlock chassisGeneralConfigBlock = chassisCalibrationFile.getBlock("GeneralInformation");


        switch (Robot.CURRENT_ROBOT_MODE) {
            case REAL -> {
                // Real robot, instantiate hardware IO implementations
                powerDistribution = new PowerDistribution(1, PowerDistribution.ModuleType.kRev);
                 drive = new SwerveDrive(
                         new GyroIOPigeon2(),
                         new ModuleIOTalonFX(chassisCalibrationFile.getBlock("FrontLeft"), chassisGeneralConfigBlock),
                         new ModuleIOTalonFX(chassisCalibrationFile.getBlock("FrontRight"), chassisGeneralConfigBlock),
                         new ModuleIOTalonFX(chassisCalibrationFile.getBlock("BackLeft"), chassisGeneralConfigBlock),
                         new ModuleIOTalonFX(chassisCalibrationFile.getBlock("BackRight"), chassisGeneralConfigBlock),
                         chassisGeneralConfigBlock
                 );

                 aprilTagVision = new AprilTagVision(
                         new AprilTagVisionIOReal(camerasProperties),
                         camerasProperties,
                         drive
                 );

                 this.shooter = new Shooter(new FlyWheelsIOReal(), new PitchIOReal());

                 this.intake = new Intake(new IntakeIOReal());
            }

            case SIM -> {
                powerDistribution = new PowerDistribution();
                // Sim robot, instantiate physics sim IO implementations
                final ModuleIOSim
                        frontLeft = new ModuleIOSim(chassisGeneralConfigBlock),
                        frontRight = new ModuleIOSim(chassisGeneralConfigBlock),
                        backLeft = new ModuleIOSim(chassisGeneralConfigBlock),
                        backRight = new ModuleIOSim(chassisGeneralConfigBlock);
                final GyroIOSim gyroIOSim = new GyroIOSim();
                drive = new SwerveDrive(
                        gyroIOSim,
                        frontLeft, frontRight, backLeft, backRight,
                        chassisGeneralConfigBlock
                );
                final SwerveDriveSimulation driveSimulation = new SwerveDriveSimulation(
                        chassisGeneralConfigBlock,
                        gyroIOSim,
                        frontLeft, frontRight, backLeft, backRight,
                        drive.kinematics,
                        new Pose2d(3, 3, new Rotation2d()),
                        drive::setPose
                );
                fieldSimulation = new Crescendo2024FieldSimulation(driveSimulation);

                aprilTagVision = new AprilTagVision(
                        new ApriltagVisionIOSim(
                                camerasProperties,
                                Constants.VisionConfigs.fieldLayout,
                                driveSimulation::getObjectOnFieldPose2d
                        ),
                        camerasProperties,
                        drive
                );

                fieldSimulation.placeGamePiecesOnField();

                fieldSimulation.addRobot(new OpponentRobotSimulation(0));
                fieldSimulation.addRobot(new OpponentRobotSimulation(1));
                fieldSimulation.addRobot(new OpponentRobotSimulation(2));

                // no shooters simulation (for now)
                shooter = new Shooter(shooterInputs -> {}, pitchInputs -> {});

                this.intake = new Intake(inputs -> {});
            }

            default -> {
                powerDistribution = new PowerDistribution();
                // Replayed robot, disable IO implementations
                drive = new SwerveDrive(
                        (inputs) -> {},
                        (inputs) -> {},
                        (inputs) -> {},
                        (inputs) -> {},
                        (inputs) -> {},
                        chassisGeneralConfigBlock
                );

                aprilTagVision = new AprilTagVision(
                        (inputs) -> {},
                        camerasProperties,
                        drive
                );

                this.shooter = new Shooter(shooterInputs -> {}, pitchInputs -> {});

                this.intake = new Intake(inputs -> {});
            }
        }
        SmartDashboard.putData("Select Test", testChooser = TestBuilder.buildTestsChooser(this));
        autoChooser = AutoBuilder.buildAutoChooser(this);

        driverModeChooser = new LoggedDashboardChooser<>("Driver Mode", new SendableChooser<>());
        driverModeChooser.addDefaultOption(DriverMode.LEFT_HANDED.name(), DriverMode.LEFT_HANDED);
        driverModeChooser.addOption(DriverMode.RIGHT_HANDED.name(), DriverMode.RIGHT_HANDED);

        configureButtonBindings();
    }

    private boolean isDSPresentedAsRed = Constants.isSidePresentedAsRed();
    private boolean isLeftHanded = true;
    /**
     * reconfigures button bindings if alliance station has changed
     * */
    public void rebindKeysIfChanged() {
        final boolean isLeftHandedSelected = !DriverMode.RIGHT_HANDED.equals(driverModeChooser.get());
        if (Constants.isSidePresentedAsRed() != isDSPresentedAsRed || isLeftHanded != isLeftHandedSelected)
            configureButtonBindings();
        isDSPresentedAsRed = Constants.isSidePresentedAsRed();
        isLeftHanded = isLeftHandedSelected;
    }
    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * Joystick} or {@link XboxController}), and then passing it to a {@link
     * JoystickButton}.
     */
    private void configureButtonBindings() {
        System.out.println("configuring key bindings...  mode:" + driverModeChooser.get());
        final MapleJoystickDriveInput driveInput = DriverMode.RIGHT_HANDED.equals(driverModeChooser.get()) ?
                MapleJoystickDriveInput.rightHandedJoystick(driverController)
                : MapleJoystickDriveInput.leftHandedJoystick(driverController);

        final Command joystickDrive = new JoystickDrive(
                driveInput,
                () -> true,
                drive
        );
        drive.setDefaultCommand(joystickDrive);

        driverController.x().whileTrue(Commands.run(drive::lockChassisWithXFormation, drive));
        driverController.start().onTrue(Commands.runOnce(
                () -> drive.setPose(new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                drive
                ).ignoringDisable(true)
        );

        driverController.y().onTrue(new PrepareToAmp(shooter, intake)).onFalse(new AmpManual(shooter, intake));

        driverController.a().whileTrue(Commands.run(intake::runReverse, intake));

        driverController.leftBumper().whileTrue(new GrabNoteManual(shooter, intake));

        final MapleShooterOptimization shooterOptimization = new MapleShooterOptimization(
                "MainShooter",
                new double[] {2, 3, 4, 4.5},
                new double[] {48, 38, 32, 28},
                new double[] {3000, 3500, 4500, 5000},
                new double[] {0.25, 0.35, 0.45, 0.5}
        );

        final JoystickDriveAndAimAtTarget faceTargetWhileDriving = new JoystickDriveAndAimAtTarget(
                driveInput, drive,
                () -> Constants.toCurrentAllianceTranslation(Constants.CrescendoField2024Constants.SPEAKER_AIM_POSITION_BLUE),
                shooterOptimization,
                1
        );
        driverController.rightBumper().whileTrue(faceTargetWhileDriving.alongWith(new SemiAutoShootSequence(
                shooter, intake, shooterOptimization, drive,
                () -> Constants.toCurrentAllianceTranslation(Constants.CrescendoField2024Constants.SPEAKER_AIM_POSITION_BLUE),
                () -> false // never shoot
        )).andThen(joystickDrive::initialize)); // reset rotation maintenance

        final JoystickDriveAndAimAtTarget faceTargetPrepareToShootWhileDriving = new JoystickDriveAndAimAtTarget(
                driveInput, drive,
                () -> Constants.toCurrentAllianceTranslation(Constants.CrescendoField2024Constants.SPEAKER_AIM_POSITION_BLUE),
                shooterOptimization,
                0.7
        );
        driverController.rightTrigger(0.5).whileTrue(faceTargetPrepareToShootWhileDriving.alongWith(new SemiAutoShootSequence(
                shooter, intake, shooterOptimization, drive,
                () -> Constants.toCurrentAllianceTranslation(Constants.CrescendoField2024Constants.SPEAKER_AIM_POSITION_BLUE),
                faceTargetPrepareToShootWhileDriving::chassisRotationInPosition
        )).andThen(joystickDrive::initialize)); // reset rotation maintenance

//        final Command pathFindToShootingPose = new AutoAlignment(
//                drive,
//                () -> Constants.toCurrentAlliancePose(new Pose2d(4, 6.4, Rotation2d.fromDegrees(-165))),
//                () -> Constants.toCurrentAlliancePose(new Pose2d(4, 6.4, Rotation2d.fromDegrees(-165))),
//                new Pose2d(0.3, 0.3, Rotation2d.fromDegrees(5)),
//                0.5
//        );
//        driverController.rightTrigger(0.5).whileTrue(pathFindToShootingPose.alongWith(new ShootAtPositionSequence(
//                shooter, intake, shooterOptimization, drive,
//                () -> Constants.toCurrentAllianceTranslation(Constants.CrescendoField2024Constants.SPEAKER_AIM_POSITION_BLUE),
//                () -> Constants.toCurrentAllianceTranslation(new Translation2d(4, 6.4)),
//                pathFindToShootingPose::isFinished
//        )));

        CommandScheduler.getInstance().schedule(Commands.run(
                () -> Logger.recordOutput(
                        "TargetDistance",
                        Constants.toCurrentAllianceTranslation(Constants.CrescendoField2024Constants.SPEAKER_AIM_POSITION_BLUE)
                                .minus(drive.getPose().getTranslation()).getNorm())
        ).ignoringDisable(true));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        final Pose2d startingPose = Constants.toCurrentAlliancePose(
                autoChooser.get().getStartingPoseAtBlueAlliance()
        );
        drive.setPose(startingPose);
        if (fieldSimulation != null)
            fieldSimulation.getMainRobot().setSimulationWorldPose(startingPose);
        return autoChooser.get();
    }


    public Command getTestCommand() {
      return testChooser.getSelected().get();
    }

    public void updateSimulationWorld() {
        if (fieldSimulation != null)
            fieldSimulation.updateSimulationWorld();
    }
}
