// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.DistanceAndAngleCalcSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IntakeRotateSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PASubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystemFront;
import frc.robot.subsystems.VisionSubsystemLeft;
import frc.robot.subsystems.VisionSubsystemRight;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

        //XK-80 HID keypad
    private final XboxController m_operator1Controller = new XboxController(1);
    private JoystickButton Button_1 = new JoystickButton(m_operator1Controller, 1);
    private JoystickButton Button_2 = new JoystickButton(m_operator1Controller, 2);
    private JoystickButton Button_3 = new JoystickButton(m_operator1Controller, 3);
    private JoystickButton Button_4 = new JoystickButton(m_operator1Controller, 4);
    private JoystickButton Button_5 = new JoystickButton(m_operator1Controller, 5);
    private JoystickButton Button_6 = new JoystickButton(m_operator1Controller, 6);
    private JoystickButton Button_7 = new JoystickButton(m_operator1Controller, 7);
    private JoystickButton Button_8 = new JoystickButton(m_operator1Controller, 8);
    private JoystickButton Button_9 = new JoystickButton(m_operator1Controller, 9);
    private JoystickButton Button_10 = new JoystickButton(m_operator1Controller, 10);
    private JoystickButton Button_11 = new JoystickButton(m_operator1Controller, 11);
    private JoystickButton Button_12 = new JoystickButton(m_operator1Controller, 12);
    private JoystickButton Button_13 = new JoystickButton(m_operator1Controller, 13);
    private JoystickButton Button_14 = new JoystickButton(m_operator1Controller, 14);
    private JoystickButton Button_15 = new JoystickButton(m_operator1Controller, 15);
    private JoystickButton Button_16 = new JoystickButton(m_operator1Controller, 16);
    private JoystickButton Button_17 = new JoystickButton(m_operator1Controller, 17);
    private JoystickButton Button_18 = new JoystickButton(m_operator1Controller, 18);
    private JoystickButton Button_19 = new JoystickButton(m_operator1Controller, 19);
    private JoystickButton Button_20 = new JoystickButton(m_operator1Controller, 20);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    //subsystems used
    public final ClimberSubsystem climbersubsystem = new ClimberSubsystem();
    public final FeederSubsystem feedersubsystem = new FeederSubsystem();
    public final IntakeRotateSubsystem intakerotatesubsystem = new IntakeRotateSubsystem();
    public final IntakeSubsystem intakesubsystem = new IntakeSubsystem();
    public final PASubsystem pasubsystem = new PASubsystem();
    public final HoodSubsystem hoodsubsystem = new HoodSubsystem();
    public final TurretSubsystem turretsubsystem = new TurretSubsystem();
    public final ShooterSubsystem shootersubsystem = new ShooterSubsystem();
    public final DistanceAndAngleCalcSubsystem distanceandanglecalcsubsystem = 
        new DistanceAndAngleCalcSubsystem(drivetrain, hoodsubsystem, shootersubsystem, turretsubsystem);

    
    //MUST BE AFTER THE DRIVETRAIN DECLERATION !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    public VisionSubsystemFront visionsubsystemfront = new VisionSubsystemFront(drivetrain);
    public VisionSubsystemLeft visionsubsystemleft = new VisionSubsystemLeft(drivetrain);
    public VisionSubsystemRight visionsubsystemright = new VisionSubsystemRight(drivetrain);
    
    //triggers
    private final Trigger t_shooterInRange = shootersubsystem.t_shooterInRange(constants.kg_ShooterSubsystem.k_SpinUpThreshold);
    private final Trigger t_hoodInRange = hoodsubsystem.t_hoodInRange(constants.kg_HoodSubsystem.k_threshold);
    private final Trigger t_turretInRange = turretsubsystem.t_turretInRange(constants.kg_TurretSubsystem.k_threshold);
    private final Trigger t_intakeIsOut = intakerotatesubsystem.t_intakeIsOut(constants.kg_IntakeRotateSubsystem.k_threshold);
    private final Trigger t_intakeIsIn = intakerotatesubsystem.t_intakeIsIn(constants.kg_IntakeRotateSubsystem.k_threshold);
    private final Trigger t_feederOn = feedersubsystem.t_feederOn(constants.kg_FeederSubsystem.k_FeederOnThreshold);
    private final Trigger t_paStopped = pasubsystem.t_paStopped(constants.kg_PASubsystem.k_StopThreshold);
   

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        configureBindings();
        namedcommands(); //pathplanner namedcommands

        autoChooser = AutoBuilder.buildAutoChooser("Exist");
        SmartDashboard.putData("Auto Mode", autoChooser);

        // Warmup PathPlanner to avoid Java pauses
        CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());

        if (isAllianceRed() == 1) {
            drivetrain.resetPose(new Pose2d(constants.kg_TargetsAndField.k_FieldLength, constants.kg_TargetsAndField.k_FieldWidth, new Rotation2d(Math.toRadians(180))));
        } else {
            drivetrain.resetPose(new Pose2d(0, 0, new Rotation2d(0)));
        }
    }

    private void configureBindings() {
        //Vision stuff
        visionsubsystemfront.addLimelightPose.whileTrue(visionsubsystemfront.addMegaTag2(() -> {return drivetrain;}));
        visionsubsystemleft.addLimelightPose.whileTrue(visionsubsystemleft.addMegaTag2(() -> {return drivetrain;}));
        visionsubsystemright.addLimelightPose.whileTrue(visionsubsystemright.addMegaTag2(() -> {return drivetrain;}));


        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * getCalculatedMaxSpeed()) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * getCalculatedMaxSpeed()) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        drivetrain.registerTelemetry(logger::telemeterize);

        joystick.pov(0).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0.4).withVelocityY(0))
        );
        joystick.pov(90).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0).withVelocityY(-.4))
        );
        joystick.pov(180).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(-0.4).withVelocityY(0))
        );
        joystick.pov(270).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0).withVelocityY(.4))
        );

        joystick.x().whileTrue(drivetrain.applyRequest(() -> brake));

        //reset bot pose
        joystick.start().onTrue(visionsubsystemfront.resetPose());
        
        //shoot
        joystick.rightTrigger()
            .and(t_hoodInRange)
            .and(t_shooterInRange)
            .and(t_turretInRange)
                .whileTrue(feedersubsystem.feedCommand()
                    .alongWith(Commands.waitUntil(t_feederOn))
                    .andThen(pasubsystem.feedCommand()
                        .alongWith(intakesubsystem.intakeCommand())
                    .withName("Shooting")))
                .onFalse(pasubsystem.stopCommand()
                    .alongWith(Commands.waitUntil(t_paStopped))
                    .andThen(feedersubsystem.stopCommand()
                        .alongWith(intakesubsystem.stopCommand())
                    .withName("Stop Shooter"))
                );
        
        //intake 
        joystick.rightBumper()
            .and(t_intakeIsOut)
                .whileTrue(intakesubsystem.intakeCommand()
                    .withName("Intaking"))
                .whileFalse(intakesubsystem.stopCommand()
                    .withName("Intake Stopping"));
            
        //intake out
        Button_1.onChange(intakerotatesubsystem.setOutCommand()
            .alongWith(Commands.waitUntil(t_intakeIsOut))
            .andThen(turretsubsystem.setHome(false))
            .alongWith(hoodsubsystem.setHome(false))
            .withName("Intake Out"));
            
        //intake in
        Button_2.onChange(turretsubsystem.setHome(true)
            .alongWith(hoodsubsystem.setHome(true))
            .alongWith(Commands.waitUntil(t_turretInRange))
            .andThen(intakerotatesubsystem.setInCommand())
            .withName("Intake In"));
         
        //climber buttons
        Button_3.whileTrue(climbersubsystem.setUpPositionCommand()
            .alongWith(turretsubsystem.setHome(true))
            .alongWith(hoodsubsystem.setHome(true))
            .alongWith(Commands.waitUntil(t_turretInRange))
            .andThen(intakerotatesubsystem.setInCommand())
            .withName("Climber Up"));
        Button_4.whileTrue(climbersubsystem.setDownPositionCommand()
            .alongWith(turretsubsystem.setHome(true))
            .alongWith(hoodsubsystem.setHome(true))
            .alongWith(Commands.waitUntil(t_turretInRange))
            .andThen(intakerotatesubsystem.setInCommand())
            .withName("Climber Down"));

        //Kill commands
        Button_5.onTrue(shootersubsystem.killCommand()
            .withName("Kill Shooter"));
        Button_6.onTrue(hoodsubsystem.killCommand()
            .withName("Kill Hood"));
        Button_7.onTrue(turretsubsystem.killCommand()
            .withName("Kill Turret"));
        Button_8.onTrue(pasubsystem.killCommand()
            .withName("Kill PA"));
        Button_9.onTrue(feedersubsystem.killCommand()
            .withName("Kill Feeder"));
        Button_10.onTrue(intakesubsystem.killCommand()
            .withName("Kill Intake"));
        Button_11.onTrue(intakerotatesubsystem.killCommand()
            .withName("Kill Intake Rotate"));
        Button_12.onTrue(shootersubsystem.killCommand()
            .alongWith(hoodsubsystem.killCommand())
            .alongWith(turretsubsystem.killCommand())
            .alongWith(pasubsystem.killCommand())
            .alongWith(feedersubsystem.killCommand())
            .alongWith(intakesubsystem.killCommand())
            .alongWith(intakerotatesubsystem.killCommand())
            .withName("Kill All"));

        //wiggle command
        Button_13.whileTrue(intakerotatesubsystem.wiggleCommand()
            .withName("Wiggle Wiggle"))
            .onFalse(intakerotatesubsystem.setOutCommand());

        //Reverse Commands
        Button_14.whileTrue(feedersubsystem.antiJamCommand()
            .withName("Reverse Feeder"))
            .onFalse(feedersubsystem.stopCommand());
        Button_15.whileTrue(intakesubsystem.antiJamCommand()
            .withName("Reverse Intake"))
            .onFalse(intakesubsystem.stopCommand());
        Button_16.whileTrue(pasubsystem.antiJamCommand()
            .withName("Reverse PA"))
            .onFalse(pasubsystem.stopCommand());
        Button_17.whileTrue(feedersubsystem.antiJamCommand()
            .alongWith(intakesubsystem.antiJamCommand())
            .alongWith(pasubsystem.antiJamCommand())
            .withName("Reverse Everything"))
            .onFalse(feedersubsystem.stopCommand()
            .alongWith(intakesubsystem.stopCommand())
            .alongWith(pasubsystem.stopCommand()));

        //home commands 
        Button_18.whileTrue(climbersubsystem.homeCommand()
            .withName("Home Climber"));
        Button_19.whileTrue(hoodsubsystem.homeCommand()
            .withName("Home Hood"));
        Button_20.whileTrue(intakerotatesubsystem.homeCommand()
            .withName("Home Intake Rotate"));
        
    }

    private void namedcommands() {//check if these all works then delete this comment
        //shoot commands
        NamedCommands.registerCommand("Shoot And Wiggle", feedersubsystem.feedCommand()
                    .alongWith(Commands.waitUntil(t_feederOn))
                    .andThen(pasubsystem.feedCommand()
                        .alongWith(intakesubsystem.intakeCommand())
                        .alongWith(intakerotatesubsystem.wiggleCommand())
                    .withName("Shooting")));
        NamedCommands.registerCommand("Stop Shoot And Wiggle", pasubsystem.stopCommand()
                    .alongWith(Commands.waitUntil(t_paStopped))
                    .andThen(feedersubsystem.stopCommand()
                        .alongWith(intakesubsystem.stopCommand())
                        .alongWith(intakerotatesubsystem.setOutCommand())
                    .withName("Stop Shooter")));

        //intake commands
        NamedCommands.registerCommand("Intake Out", intakerotatesubsystem.setOutCommand().alongWith(Commands.waitUntil(t_intakeIsOut))
            .andThen(turretsubsystem.setHome(false))
            .alongWith(hoodsubsystem.setHome(false))
            .withName("Intake Out"));
        NamedCommands.registerCommand("Intake In", turretsubsystem.setHome(true)
            .alongWith(hoodsubsystem.setHome(true))
            .alongWith(Commands.waitUntil(t_turretInRange))
            .andThen(intakerotatesubsystem.setInCommand())
            .withName("Intake In"));
        NamedCommands.registerCommand("Intake On", Commands.waitUntil(t_intakeIsOut)
            .andThen(intakesubsystem.intakeCommand()));
        NamedCommands.registerCommand("Intake Stop", intakesubsystem.stopCommand());

        //Climb commands
        NamedCommands.registerCommand("Climber Up", climbersubsystem.setUpPositionCommand()
            .alongWith(turretsubsystem.setHome(true))
            .alongWith(hoodsubsystem.setHome(true))
            .alongWith(Commands.waitUntil(t_turretInRange))
            .andThen(intakerotatesubsystem.setInCommand())
            .withName("Climber Up"));
        NamedCommands.registerCommand("Climber Down", climbersubsystem.setDownPositionCommand()
            .alongWith(turretsubsystem.setHome(true))
            .alongWith(hoodsubsystem.setHome(true))
            .alongWith(Commands.waitUntil(t_turretInRange))
            .andThen(intakerotatesubsystem.setInCommand())
            .withName("Climber Down"));
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }

    private static int isAllianceRed() {
        var alliance = DriverStation.getAlliance();
        if (alliance.get() == DriverStation.Alliance.Red) {
            return 1;
        } else {
            return 0;
        }
    }

    private double getCalculatedMaxSpeed(){
        if (joystick.rightTrigger().getAsBoolean()) {
            return MaxSpeed/5;
        } else {
            return MaxSpeed;
        }
    }
}
