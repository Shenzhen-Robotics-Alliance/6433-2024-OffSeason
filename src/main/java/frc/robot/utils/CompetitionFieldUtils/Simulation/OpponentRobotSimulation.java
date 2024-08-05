package frc.robot.utils.CompetitionFieldUtils.Simulation;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;
import frc.robot.commands.drive.CustomFollowPathOnFly;
import frc.robot.subsystems.drive.HolonomicDriveSubsystem;
import frc.robot.utils.MapleJoystickDriveInput;
import frc.robot.utils.MaplePathPlannerLoader;
import org.ejml.simple.UnsupportedOperation;

/**
 * simulates an opponent robot on field
 * in physics, the opponent robot behaves just the same as our own robot, it also follows the Holonomic Chassis Physics
 * the difference is, opponent robots are not controlled by the main gamepad
 * it is either controlled by another gamepad to simulate a defense robot
 * or can follow pre-generated paths to simulate opponent robots who are doing cycles
 * */
public class OpponentRobotSimulation extends HolonomicChassisSimulation implements HolonomicDriveSubsystem {
    /* if an opponent robot is not requested to be on field, it queens outside the field for performance */
    public static final Pose2d[] ROBOT_QUEENING_POSITIONS = new Pose2d[] {
            new Pose2d(-6, 0, new Rotation2d()),
            new Pose2d(-4, 0, new Rotation2d()),
            new Pose2d(-2, 0, new Rotation2d())
    };
    public static final Pose2d[] RED_ROBOTS_STARTING_POSITIONS = new Pose2d[] {
            new Pose2d(15.2, 6.5, new Rotation2d()),
            new Pose2d(15.2, 6, new Rotation2d()),
            new Pose2d(15.2, 5.5, new Rotation2d())
    };
    public static final HolonomicChassisSimulation.RobotProfile opponentRobotProfile = new RobotProfile(
            4,
            12,
            Math.toRadians(360),
            Units.lbsToKilograms(125),
            Constants.RobotPhysicsSimulationConfigs.DEFAULT_BUMPER_WIDTH_METERS,
            Constants.RobotPhysicsSimulationConfigs.DEFAULT_BUMPER_LENGTH_METERS
    );

    private final int robotID;
    private final SendableChooser<Command> behaviorChooser = new SendableChooser<>();
    private final Runnable disable;
    /**
     * @param id the id of the robot, 0 to 2, this determines where the robot "respawns"
     * */
    public OpponentRobotSimulation(int id) {
        super(opponentRobotProfile, ROBOT_QUEENING_POSITIONS[id]);
        this.robotID = id;
        this.disable = () -> {
            stop();
            setSimulationWorldPose(ROBOT_QUEENING_POSITIONS[robotID]);
        };

        behaviorChooser.setDefaultOption("Disabled", Commands.runOnce(disable, this));
        behaviorChooser.addOption("Auto Cycle", getAutoCyleRepeadtelyCommand());
        final XboxController xboxController = new XboxController(1+robotID);
        behaviorChooser.addOption(
                "Joystick Control Left-Handed",
                getJoystickDrive(MapleJoystickDriveInput.leftHandedJoystick(xboxController))
        );
        behaviorChooser.addOption(
                "Joystick Control Right-Handed",
                getJoystickDrive(MapleJoystickDriveInput.rightHandedJoystick(xboxController))
        );
        behaviorChooser.onChange((selectedCommand) -> CommandScheduler.getInstance().schedule(selectedCommand));

        SmartDashboard.putData("FieldSimulation/OpponentRobot"+robotID+" Behavior", behaviorChooser);
    }

    private ChassisSpeeds speedSetPoint = new ChassisSpeeds();
    @Override
    public void runRawChassisSpeeds(ChassisSpeeds speeds) {
        this.speedSetPoint = speeds;
    }

    @Override
    public Pose2d getPose() {
        return super.getObjectOnFieldPose2d();
    }

    @Override
    public void setPose(Pose2d currentPose) {
        super.setSimulationWorldPose(currentPose);
    }

    @Override public double getChassisMaxLinearVelocityMetersPerSec() {return profile.robotMaxVelocity;}
    @Override public double getChassisMaxAccelerationMetersPerSecSq() {return profile.robotMaxAcceleration;}
    @Override public double getChassisMaxAngularVelocity() {return profile.maxAngularVelocity;}
    @Override public double getChassisMaxAngularAccelerationRadPerSecSq() {return profile.maxAngularAcceleration;}

    @Override public void addVisionMeasurement(Pose2d visionPose, double timestamp, Matrix<N3, N1> stdDevs) {throw new UnsupportedOperation("an opponent robot does not support vision measurement"); }
    @Override public void updateSimulationSubPeriod(int iterationNum, double subPeriodSeconds) {
        super.simulateChassisBehaviorWithRobotRelativeSpeeds(speedSetPoint);
    }

    private static final PathConstraints constraints = new PathConstraints(3.5, 8, Math.toRadians(180), Math.toRadians(360));
    private static final Pose2d tolerance = new Pose2d(3, 3, Rotation2d.fromDegrees(20));
    public Command getAutoCyleRepeadtelyCommand() {
        final PathPlannerPath cycleForwardPath = MaplePathPlannerLoader.fromPathFile("opponent cycle path " + robotID, constraints),
                cycleBackwardPath = MaplePathPlannerLoader.fromPathFileReversed(
                        "opponent cycle path " + robotID,
                        constraints,
                        new GoalEndState(0, cycleForwardPath.getPreviewStartingHolonomicPose().getRotation()));
        final Command teleportToStartingPose = Commands.runOnce(() -> setSimulationWorldPose(cycleBackwardPath.getPreviewStartingHolonomicPose()), this),
                cycleForward = new CustomFollowPathOnFly(
                        this,
                        () -> Constants.isSidePresentedAsRed() ? cycleForwardPath.flipPath() : cycleForwardPath,
                        tolerance,
                        1,
                        false,
                        "Field/Opponent"+robotID+"Cycle"
                ),
                cycleBackWards = new CustomFollowPathOnFly(
                        this,
                        () -> Constants.isSidePresentedAsRed() ? cycleBackwardPath.flipPath() : cycleBackwardPath,
                        tolerance,
                        1,
                        false,
                        "Field/Opponent"+robotID+"Cycle"
                );

        final Runnable end = () -> {
            stop();
            setSimulationWorldPose(ROBOT_QUEENING_POSITIONS[robotID]);
        };

        final Command cycleRepeatedlyAndStop = new SequentialCommandGroup(
                teleportToStartingPose,
                new SequentialCommandGroup(
                        cycleBackWards,
                        cycleForward
                ).repeatedly()
        ).finallyDo(end);
        cycleRepeatedlyAndStop.addRequirements(this);
        return cycleRepeatedlyAndStop;
    }

    public Command getJoystickDrive(MapleJoystickDriveInput joystickDriveInput) {
        final Pose2d startingPose = Constants.toCurrentAlliancePose(RED_ROBOTS_STARTING_POSITIONS[robotID]),
                queeningPose = ROBOT_QUEENING_POSITIONS[robotID];
        final Command teleportToStartingPose = Commands.runOnce(() -> setSimulationWorldPose(startingPose), this);
        Runnable end = () -> {
            setSimulationWorldPose(queeningPose);
            stop();
        };

        return new SequentialCommandGroup(
                teleportToStartingPose,
                Commands.run(() -> joystickDrivePeriod(joystickDriveInput), this)
        ).finallyDo(end);
    }

    private void joystickDrivePeriod(MapleJoystickDriveInput driveInput) {
        final ChassisSpeeds gamePadSpeeds = driveInput.getJoystickChassisSpeeds(4, 8),
                gamePadSpeedsInOurDriverStationReference = ChassisSpeeds.fromFieldRelativeSpeeds(gamePadSpeeds, new Rotation2d(Math.PI));
        HolonomicDriveSubsystem.super.runDriverStationCentricChassisSpeeds(gamePadSpeedsInOurDriverStationReference);
    }

    /**
     * a method to test the driving physics
     * just for testing
     * in the formal code, we should be using holonomic drive commands
     * */
    @Deprecated
    public void testDrivingPhysicsWithJoystick(XboxController xboxController) {
        final MapleJoystickDriveInput mapleJoystickDriveInput = MapleJoystickDriveInput.leftHandedJoystick(xboxController);
        final ChassisSpeeds gamePadSpeeds = mapleJoystickDriveInput.getJoystickChassisSpeeds(5, 10);
        HolonomicDriveSubsystem.super.runDriverStationCentricChassisSpeeds(gamePadSpeeds);
    }
}
