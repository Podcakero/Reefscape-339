// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package us.kilroyrobotics;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import java.util.List;
import us.kilroyrobotics.Constants.CameraConstants;
import us.kilroyrobotics.Constants.CoralMechanismConstants;
import us.kilroyrobotics.Constants.DriveConstants;
import us.kilroyrobotics.Constants.ElevatorConstants;
import us.kilroyrobotics.Constants.VisionConstants;
import us.kilroyrobotics.generated.TunerConstants;
import us.kilroyrobotics.subsystems.Camera;
// import us.kilroyrobotics.subsystems.AlgaeIntake;
// import us.kilroyrobotics.subsystems.AlgaeIntake.AlgaeState;
import us.kilroyrobotics.subsystems.CommandSwerveDrivetrain;
import us.kilroyrobotics.subsystems.CoralIntakeMotor;
import us.kilroyrobotics.subsystems.Elevator;
import us.kilroyrobotics.subsystems.LEDs;
import us.kilroyrobotics.subsystems.LEDs.LEDMode;
import us.kilroyrobotics.subsystems.Tower;
import us.kilroyrobotics.subsystems.Wrist;
import us.kilroyrobotics.util.LimelightHelpers;
import us.kilroyrobotics.util.LimelightHelpers.RawFiducial;
import us.kilroyrobotics.util.TowerEvent;
import us.kilroyrobotics.util.TowerState;

public class RobotContainer {
    private LinearVelocity currentDriveSpeed = DriveConstants.kMediumDriveSpeed;
    private double kMaxAngularRate =
            RotationsPerSecond.of(0.3).in(RadiansPerSecond); // 1/3 of a rotation per second
    // max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive =
            new SwerveRequest.FieldCentric()
                    .withDeadband(currentDriveSpeed.in(MetersPerSecond) * 0.1)
                    .withRotationalDeadband(kMaxAngularRate * 0.1) // Add a 10% deadband
                    .withDriveRequestType(
                            DriveRequestType
                                    .OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight =
            new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(currentDriveSpeed.in(MetersPerSecond));

    /* Controllers */
    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandJoystick leftOperatorJoystick = new CommandJoystick(1);
    private final CommandJoystick rightOperatorJoystick = new CommandJoystick(2);

    /* Subsystems */
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public final CoralIntakeMotor coralIntakeMotor = new CoralIntakeMotor();

    @Logged(name = "Elevator")
    public final Elevator elevator = new Elevator();

    @Logged(name = "Wrist")
    public final Wrist wrist = new Wrist(elevator::getCarriagePose, Robot.isReal());

    public final LEDs leds = new LEDs();

    public final Tower tower = new Tower(elevator, wrist, coralIntakeMotor, leds);

    /* Commands */
    public final Command intakeCoral =
            Commands.runOnce(
                    () -> tower.triggerEvent(TowerEvent.INTAKE_CORAL),
                    tower,
                    elevator,
                    wrist,
                    coralIntakeMotor);
    private final Command scoreCoral =
            Commands.runOnce(
                    () -> tower.triggerEvent(TowerEvent.SCORE),
                    tower,
                    elevator,
                    wrist,
                    coralIntakeMotor);
    private final Command gotoL1 =
            Commands.runOnce(
                    () -> tower.triggerEvent(TowerEvent.GOTO_L1),
                    tower,
                    elevator,
                    wrist,
                    coralIntakeMotor);
    private final Command gotoL2 =
            Commands.runOnce(
                    () -> tower.triggerEvent(TowerEvent.GOTO_L2),
                    tower,
                    elevator,
                    wrist,
                    coralIntakeMotor);
    private final Command gotoL3 =
            Commands.runOnce(
                    () -> tower.triggerEvent(TowerEvent.GOTO_L3),
                    tower,
                    elevator,
                    wrist,
                    coralIntakeMotor);
    private final Command gotoL4 =
            Commands.runOnce(
                    () -> tower.triggerEvent(TowerEvent.GOTO_L4),
                    tower,
                    elevator,
                    wrist,
                    coralIntakeMotor);
    private final Command gotoHome =
            Commands.runOnce(
                    () -> tower.triggerEvent(TowerEvent.HOME_TOWER),
                    tower,
                    elevator,
                    wrist,
                    coralIntakeMotor);

    /* Manual Control Commands */
    private final Command initTower =
            Commands.runOnce(
                    () -> tower.setState(TowerState.INIT),
                    tower,
                    elevator,
                    wrist,
                    coralIntakeMotor);
    private final Command raiseToL1 =
            Commands.runOnce(
                    () -> tower.setState(TowerState.RAISING_TO_L1),
                    tower,
                    elevator,
                    wrist,
                    coralIntakeMotor);
    private final Command raiseToL2 =
            Commands.runOnce(
                    () -> tower.setState(TowerState.RAISING_TO_L2),
                    tower,
                    elevator,
                    wrist,
                    coralIntakeMotor);
    private final Command raiseToL3 =
            Commands.runOnce(
                    () -> tower.setState(TowerState.RAISING_TO_L3),
                    tower,
                    elevator,
                    wrist,
                    coralIntakeMotor);
    private final Command raiseToL4 =
            Commands.runOnce(
                    () -> tower.setState(TowerState.RAISING_TO_L4),
                    tower,
                    elevator,
                    wrist,
                    coralIntakeMotor);
    private final Command tiltToIntake =
            Commands.runOnce(
                    () -> tower.setState(TowerState.TILTING_TO_INTAKE),
                    tower,
                    elevator,
                    wrist,
                    coralIntakeMotor);

    private final Command controlWrist =
            Commands.run(
                    () ->
                            wrist.setSpeed(
                                    -leftOperatorJoystick.getY()
                                            * CoralMechanismConstants.kOverrideSpeedMultiplier),
                    wrist);
    private final Command controlElevator =
            Commands.run(
                    () ->
                            elevator.setSpeed(
                                    rightOperatorJoystick.getY()
                                            * ElevatorConstants.kOverrideSpeedMultiplier),
                    elevator);
    private final Command wristStop =
            Commands.runOnce(
                    () -> {
                        wrist.stop();
                        wrist.resetClosedLoopControl();
                    },
                    wrist);
    private final Command elevatorStop =
            Commands.runOnce(
                    () -> {
                        elevator.stop();
                        elevator.resetClosedLoopControl();
                    },
                    elevator);

    /* Drivetrain Control Commands */
    private final Command defenseMode =
            Commands.runOnce(
                    () -> {
                        boolean defenseModeOn = SmartDashboard.getBoolean("DefenseModeOn", true);
                        currentDriveSpeed =
                                defenseModeOn
                                        ? DriveConstants.kMediumDriveSpeed
                                        : DriveConstants.kHighDriveSpeed;
                        SmartDashboard.putBoolean("DefenseModeOn", !defenseModeOn);
                    });

    private int currentAprilTag = 0;

    /* Reef Alignment */
    private final Command alignReefLeft =
            Commands.runOnce(
                    () -> {
                        Pose2d targetPose;

                        RawFiducial[] aprilTags =
                                LimelightHelpers.getRawFiducials("limelight-right");
                        if (aprilTags.length < 1)
                            aprilTags = LimelightHelpers.getRawFiducials("limelight-left");

                        if (aprilTags.length < 1) {
                            if (currentAprilTag == 0) return;

                            targetPose =
                                    VisionConstants.getAlignmentPose(
                                            currentAprilTag,
                                            true,
                                            DriverStation.getAlliance().orElse(Alliance.Blue));
                        } else {
                            RawFiducial aprilTag = aprilTags[0];
                            currentAprilTag = aprilTag.id;
                            System.out.println(
                                    "[TELEOP-ASSIST] "
                                            + " Selected tag "
                                            + currentAprilTag
                                            + " for alignment");

                            targetPose =
                                    VisionConstants.getAlignmentPose(
                                            aprilTag.id,
                                            true,
                                            DriverStation.getAlliance().orElse(Alliance.Blue));
                        }

                        if (targetPose == null) return;

                        System.out.println(
                                "[TELEOP-ASSIST] " + "[LEFT]" + " Going to pose " + targetPose);

                        List<Waypoint> waypoints =
                                PathPlannerPath.waypointsFromPoses(
                                        drivetrain.getState().Pose, targetPose);

                        PathConstraints constraints = new PathConstraints(1.5, 1.0, 0.75, 0.5);

                        PathPlannerPath path =
                                new PathPlannerPath(
                                        waypoints,
                                        constraints,
                                        null,
                                        new GoalEndState(0.0, targetPose.getRotation()));
                        path.preventFlipping = true;

                        CommandScheduler.getInstance()
                                .schedule(
                                        Commands.sequence(
                                                Commands.runOnce(
                                                        () -> {
                                                            leds.setMode(LEDMode.Off);
                                                            SmartDashboard.putBoolean(
                                                                    "TeleopAlignIndicator", false);
                                                        }),
                                                AutoBuilder.followPath(path),
                                                Commands.runOnce(
                                                        () -> {
                                                            System.out.println(
                                                                    "[TELEOP-ASSIST] "
                                                                            + "[LEFT]"
                                                                            + " Arrived at Pose for tag "
                                                                            + currentAprilTag);
                                                            currentAprilTag = 0;

                                                            SmartDashboard.putBoolean(
                                                                    "TeleopAlignIndicator", true);
                                                            leds.setMode(LEDMode.TeleopAligned);

                                                            CommandScheduler.getInstance()
                                                                    .schedule(
                                                                            Commands.sequence(
                                                                                    new WaitCommand(
                                                                                            2.5),
                                                                                    Commands
                                                                                            .runOnce(
                                                                                                    () -> {
                                                                                                        SmartDashboard
                                                                                                                .putBoolean(
                                                                                                                        "TeleopAlignIndicator",
                                                                                                                        false);
                                                                                                        this
                                                                                                                .leds
                                                                                                                .setMode(
                                                                                                                        LEDMode
                                                                                                                                .Off);
                                                                                                    })));
                                                        })));
                    });

    private final Command alignReefRight =
            Commands.runOnce(
                    () -> {
                        Pose2d targetPose;

                        RawFiducial[] aprilTags =
                                LimelightHelpers.getRawFiducials("limelight-right");
                        if (aprilTags.length < 1)
                            aprilTags = LimelightHelpers.getRawFiducials("limelight-left");

                        if (aprilTags.length < 1) {
                            if (currentAprilTag == 0) return;

                            targetPose =
                                    VisionConstants.getAlignmentPose(
                                            currentAprilTag,
                                            false,
                                            DriverStation.getAlliance().orElse(Alliance.Blue));
                        } else {
                            RawFiducial aprilTag = aprilTags[0];
                            currentAprilTag = aprilTag.id;
                            System.out.println(
                                    "[TELEOP-ASSIST] "
                                            + " Selected tag "
                                            + currentAprilTag
                                            + " for alignment");

                            targetPose =
                                    VisionConstants.getAlignmentPose(
                                            aprilTag.id,
                                            false,
                                            DriverStation.getAlliance().orElse(Alliance.Blue));
                        }

                        if (targetPose == null) return;

                        System.out.println(
                                "[TELEOP-ASSIST] " + "[RIGHT]" + " Going to pose " + targetPose);

                        List<Waypoint> waypoints =
                                PathPlannerPath.waypointsFromPoses(
                                        drivetrain.getState().Pose, targetPose);

                        PathConstraints constraints = new PathConstraints(1.5, 1.0, 0.75, 0.5);

                        PathPlannerPath path =
                                new PathPlannerPath(
                                        waypoints,
                                        constraints,
                                        null,
                                        new GoalEndState(0.0, targetPose.getRotation()));
                        path.preventFlipping = true;

                        CommandScheduler.getInstance()
                                .schedule(
                                        Commands.sequence(
                                                Commands.runOnce(
                                                        () -> {
                                                            leds.setMode(LEDMode.Off);
                                                            SmartDashboard.putBoolean(
                                                                    "TeleopAlignIndicator", false);
                                                        }),
                                                AutoBuilder.followPath(path),
                                                Commands.runOnce(
                                                        () -> {
                                                            System.out.println(
                                                                    "[TELEOP-ASSIST] "
                                                                            + "[RIGHT]"
                                                                            + " Arrived at Pose for tag "
                                                                            + currentAprilTag);
                                                            currentAprilTag = 0;

                                                            SmartDashboard.putBoolean(
                                                                    "TeleopAlignIndicator", true);
                                                            leds.setMode(LEDMode.TeleopAligned);

                                                            CommandScheduler.getInstance()
                                                                    .schedule(
                                                                            Commands.sequence(
                                                                                    new WaitCommand(
                                                                                            2.5),
                                                                                    Commands
                                                                                            .runOnce(
                                                                                                    () -> {
                                                                                                        SmartDashboard
                                                                                                                .putBoolean(
                                                                                                                        "TeleopAlignIndicator",
                                                                                                                        false);

                                                                                                        leds
                                                                                                                .setMode(
                                                                                                                        LEDMode
                                                                                                                                .Off);
                                                                                                    })));
                                                        })));
                    });

    /* AutoLeave Command */
    private final Timer autoLeaveTimer = new Timer();
    private final Command autoLeave =
            Commands.sequence(
                            Commands.runOnce(() -> autoLeaveTimer.restart()),
                            drivetrain.applyRequest(
                                    () -> forwardStraight.withVelocityX(MetersPerSecond.of(0.5))))
                    .onlyWhile(() -> !autoLeaveTimer.hasElapsed(2));

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    @SuppressWarnings("unused")
    public RobotContainer() {
        if (Robot.isReal() && CameraConstants.kCameraEnabled) new Camera();

        NamedCommands.registerCommand("Intake Coral", intakeCoral);
        NamedCommands.registerCommand("Score Coral", scoreCoral);
        NamedCommands.registerCommand("GoTo L1", gotoL1);
        NamedCommands.registerCommand("GoTo L2", gotoL2);
        NamedCommands.registerCommand("GoTo L3", gotoL3);
        NamedCommands.registerCommand("GoTo L4", gotoL4);

        NamedCommands.registerCommand("Leave", autoLeave);

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Mode", autoChooser);

        autoChooser.onChange(
                (Command command) -> {
                    if (command != null)
                        System.out.println("[AUTO] Selected Route " + command.getName());
                });

        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(
                        () ->
                                drive.withVelocityX(
                                                -driverController.getLeftY()
                                                        * currentDriveSpeed.in(
                                                                MetersPerSecond)) // Drive forward
                                        // with
                                        // negative Y
                                        // (forward)
                                        .withVelocityY(
                                                -driverController.getLeftX()
                                                        * currentDriveSpeed.in(
                                                                MetersPerSecond)) // Drive left with
                                        // negative X
                                        // (left)
                                        .withRotationalRate(
                                                -driverController.getRightX()
                                                        * kMaxAngularRate) // Drive counterclockwise
                        // with
                        // negative X (left)
                        ));

        // Brake
        driverController.b().whileTrue(drivetrain.applyRequest(() -> brake));

        driverController
                .pov(0)
                .whileTrue(
                        drivetrain.applyRequest(
                                () ->
                                        forwardStraight
                                                .withVelocityX(DriveConstants.kLowDriveSpeed)
                                                .withVelocityY(0)));
        driverController
                .pov(45)
                .whileTrue(
                        drivetrain.applyRequest(
                                () ->
                                        forwardStraight
                                                .withVelocityX(DriveConstants.kLowDriveSpeed)
                                                .withVelocityY(
                                                        DriveConstants.kLowDriveSpeed
                                                                .unaryMinus())));
        driverController
                .pov(90)
                .whileTrue(
                        drivetrain.applyRequest(
                                () ->
                                        forwardStraight
                                                .withVelocityX(0)
                                                .withVelocityY(
                                                        DriveConstants.kLowDriveSpeed
                                                                .unaryMinus())));
        driverController
                .pov(135)
                .whileTrue(
                        drivetrain.applyRequest(
                                () ->
                                        forwardStraight
                                                .withVelocityX(
                                                        DriveConstants.kLowDriveSpeed.unaryMinus())
                                                .withVelocityY(
                                                        DriveConstants.kLowDriveSpeed
                                                                .unaryMinus())));
        driverController
                .pov(180)
                .whileTrue(
                        drivetrain.applyRequest(
                                () ->
                                        forwardStraight
                                                .withVelocityX(
                                                        DriveConstants.kLowDriveSpeed.unaryMinus())
                                                .withVelocityY(0)));
        driverController
                .pov(225)
                .whileTrue(
                        drivetrain.applyRequest(
                                () ->
                                        forwardStraight
                                                .withVelocityX(
                                                        DriveConstants.kLowDriveSpeed.unaryMinus())
                                                .withVelocityY(DriveConstants.kLowDriveSpeed)));
        driverController
                .pov(270)
                .whileTrue(
                        drivetrain.applyRequest(
                                () ->
                                        forwardStraight
                                                .withVelocityX(0)
                                                .withVelocityY(DriveConstants.kLowDriveSpeed)));
        driverController
                .pov(315)
                .whileTrue(
                        drivetrain.applyRequest(
                                () ->
                                        forwardStraight
                                                .withVelocityX(DriveConstants.kLowDriveSpeed)
                                                .withVelocityY(DriveConstants.kLowDriveSpeed)));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // driverController
        //         .back()
        //         .and(driverController.y())
        //         .whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // driverController
        //         .back()
        //         .and(driverController.x())
        //         .whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // driverController
        //         .start()
        //         .and(driverController.y())
        //         .whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // driverController
        //         .start()
        //         .and(driverController.x())
        //         .whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press
        driverController.a().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        // Switch to higher speeds (defense mode)
        driverController.start().onTrue(defenseMode);

        /* Automatic Control */
        leftOperatorJoystick.button(2).onTrue(intakeCoral);
        leftOperatorJoystick.button(3).onTrue(scoreCoral);
        leftOperatorJoystick.button(10).onTrue(gotoL1);
        leftOperatorJoystick.button(7).onTrue(gotoL2);
        leftOperatorJoystick.button(11).onTrue(gotoL3);
        leftOperatorJoystick.button(6).onTrue(gotoL4);

        /* Manual Control */
        rightOperatorJoystick.button(10).onTrue(initTower);
        rightOperatorJoystick.button(10).onTrue(raiseToL1);
        rightOperatorJoystick.button(7).onTrue(raiseToL2);
        rightOperatorJoystick.button(11).onTrue(raiseToL3);
        rightOperatorJoystick.button(6).onTrue(raiseToL4);
        rightOperatorJoystick.button(8).onTrue(tiltToIntake);
        leftOperatorJoystick.button(1).whileTrue(controlWrist).onFalse(wristStop);
        rightOperatorJoystick.button(1).whileTrue(controlElevator).onFalse(elevatorStop);

        // Reef Alignment
        driverController.leftBumper().onTrue(alignReefLeft);
        driverController.rightBumper().onTrue(alignReefRight);
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}
