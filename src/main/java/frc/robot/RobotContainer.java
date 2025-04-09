// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.filter.Debouncer;

import frc.robot.commands.ElevatorPID;
import frc.robot.commands.ElevatorPIDAuto;
import frc.robot.commands.ElevatorPIDAutoL1;
import frc.robot.commands.ElevatorPIDAutoL2;
import frc.robot.commands.AlignToReefTagRelative;
import frc.robot.commands.CoralAutoScore;
import frc.robot.commands.CoralAutoScoreFast;
import frc.robot.commands.CoralAutoStop;
import frc.robot.commands.CoralAutoPreroller;
//import frc.robot.commands.ChaseTagCommand;
import frc.robot.commands.ElevatorManual;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AlgaeRollers;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralRollers;
import frc.robot.subsystems.Climb;

import edu.wpi.first.cameraserver.CameraServer;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    private double speedLimiter = 0.5;
    private double directionFlipper = 1.0;

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 2% deadband
            .withDriveRequestType(DriveRequestType.Velocity); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.Velocity);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandXboxController subjoystick = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final Elevator elevator = new Elevator();
    private final AlgaeRollers algaeroller = new AlgaeRollers();
    private final CoralRollers coralroller = new CoralRollers();
    private final Climb climb = new Climb();

    /* Path follower */
    private SendableChooser<Command> autoChooser;

    public RobotContainer() {
        CameraServer.startAutomaticCapture();

        // register commands to pathplanner
        //NamedCommands.registerCommand("elevatorL4", new RunCommand(() -> elevator.elevatorToSetPoint(ElevatorConstants.l4)));
        //NamedCommands.registerCommand("elevatorL1", new RunCommand(() -> elevator.elevatorToSetPoint(ElevatorConstants.l1)));
        // NamedCommands.registerCommand("elevatorL1", new ElevatorPID(elevator, ElevatorConstants.l1));
        NamedCommands.registerCommand("elevatorL2", new ElevatorPIDAutoL2(elevator, ElevatorConstants.l2));
        NamedCommands.registerCommand("elevatorL4Actual", new ElevatorPIDAuto(elevator, ElevatorConstants.l4));
        NamedCommands.registerCommand("elevatorL1", new ElevatorPIDAutoL1(elevator, ElevatorConstants.l1-0.5));
        NamedCommands.registerCommand("AlignReef", new AlignToReefTagRelative(drivetrain, true));
        NamedCommands.registerCommand("scoreOut", new CoralAutoScore(coralroller));
        NamedCommands.registerCommand("scoreOutFast", new CoralAutoScoreFast(coralroller));
        NamedCommands.registerCommand("preRoller", new CoralAutoPreroller(coralroller));
        NamedCommands.registerCommand("stopCoralRollers", new CoralAutoStop(coralroller));

        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed * speedLimiter * directionFlipper) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed * speedLimiter * directionFlipper) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate * speedLimiter * 1.5) // Drive counterclockwise with negative X (left)
            )
        );

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));

        joystick.b().whileTrue(new RunCommand(() -> this.flipDirection(1.0)));
        joystick.y().whileTrue(new RunCommand(() -> this.flipDirection(-1.0)));

        // joystick.y().whileTrue(new ChaseTagCommand(drivetrain, null));

        /*
        joystick.pov(0).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0.5).withVelocityY(0))
        );
        joystick.pov(180).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        );
        */
        /* 
        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
        */
        
        //elevator coral presets
        //subjoystick.x().whileTrue(new RunCommand(() -> elevator.elevatorToSetPoint(ElevatorConstants.l1)));
        //subjoystick.y().whileTrue(new RunCommand(() -> elevator.elevatorToSetPoint(ElevatorConstants.l2)));
        //subjoystick.a().whileTrue(new RunCommand(() -> elevator.elevatorToSetPoint(ElevatorConstants.l3)));
        //subjoystick.b().whileTrue(new RunCommand(() -> elevator.elevatorToSetPoint(ElevatorConstants.l4)));
        //joystick.leftBumper().whileTrue(new RunCommand(() -> elevator.elevatorToSetPoint(ElevatorConstants.l3_5)));
        //joystick.rightBumper().whileTrue(new RunCommand(() -> elevator.elevatorToSetPoint(ElevatorConstants.l2_5)));
        subjoystick.x().onTrue(new ElevatorPID(elevator, ElevatorConstants.l1));
        subjoystick.a().onTrue(new ElevatorPID(elevator, ElevatorConstants.l2));
        subjoystick.y().onTrue(new ElevatorPID(elevator, ElevatorConstants.l3));
        subjoystick.b().onTrue(new ElevatorPID(elevator, ElevatorConstants.l4));
        joystick.leftBumper().onTrue(new ElevatorPID(elevator, ElevatorConstants.l3_5));
        joystick.rightBumper().onTrue(new ElevatorPID(elevator, ElevatorConstants.l2_5));
        
        joystick.leftStick().onTrue(new AlignToReefTagRelative(drivetrain, true));
        //subjoystick.rightStick().onTrue(new AlignToReefTagRelative(drivetrain, false));

        subjoystick.povRight().whileTrue(new RunCommand(() -> climb.pull()));
        subjoystick.povRight().debounce(0.2).onFalse(new RunCommand(() -> climb.stop()));
        subjoystick.povLeft().whileTrue(new RunCommand(() -> climb.reset()));
        subjoystick.povLeft().debounce(0.2).onFalse(new RunCommand(() -> climb.stop()));
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    
        //subjoystick.povUp().onTrue(elevator.disableElevatorPID());
        //subjoystick.povDown().onTrue(elevator.disableElevatorPID());

        //subjoystick.povUp().whileTrue(new RunCommand(() -> elevator.manualChange(0.07)));
        //subjoystick.povDown().whileTrue(new RunCommand(() -> elevator.manualChange(-0.07)));

        subjoystick.povUp().whileTrue(new ElevatorManual(elevator, ElevatorConstants.elevatorSpeed + 0.05));
        subjoystick.povUp().onFalse(new ElevatorManual(elevator, 0.01));
        subjoystick.povDown().whileTrue(new ElevatorManual(elevator, -ElevatorConstants.elevatorSpeed + 0.05));
        subjoystick.povDown().onFalse(new ElevatorManual(elevator, 0.01));

        joystick.povUp().whileTrue(new RunCommand(() -> this.setSpeed(1.0)));
        joystick.povRight().whileTrue(new RunCommand(() -> this.setSpeed(0.400)));
        joystick.povLeft().whileTrue(new RunCommand(() -> this.setSpeed(0.200)));
        joystick.povDown().whileTrue(new RunCommand(() -> this.setSpeed(0.1)));
        
        //rollers for subsystems
        subjoystick.rightBumper().whileTrue(new RunCommand(() -> coralroller.rollBack()));
        subjoystick.rightBumper().whileFalse(new RunCommand(() -> coralroller.stopCoralRollers()));
        subjoystick.leftBumper().whileFalse(new RunCommand(() -> coralroller.stopCoralRollers()));
        subjoystick.leftBumper().whileTrue(new RunCommand(() -> coralroller.preRoller()));
        subjoystick.rightStick().whileFalse(new RunCommand(() -> coralroller.stopCoralRollers()));
        subjoystick.rightStick().whileTrue(new RunCommand(() -> coralroller.scoreOut()));

        subjoystick.rightBumper().whileTrue(new RunCommand(() -> algaeroller.spinOut()));
        subjoystick.rightBumper().whileFalse(new RunCommand(() -> algaeroller.stopAlgaeRollers()));
        subjoystick.leftBumper().whileFalse(new RunCommand(() -> algaeroller.stopAlgaeRollers()));
        subjoystick.leftBumper().whileTrue(new RunCommand(() -> algaeroller.spinIn()));
    }

    public void setSpeed(double spe){
        speedLimiter = spe;
        if(spe == 0.066) SmartDashboard.putString("Swerve Speed", "CRAWL");
        if(spe == 0.2) SmartDashboard.putString("Swerve Speed", "LOW");
        if(spe == 0.5) SmartDashboard.putString("Swerve Speed", "MID");
        if(spe == 1.0) SmartDashboard.putString("Swerve Speed", "HIGH");
    }

    public void flipDirection(double newDir){
        this.directionFlipper = newDir;
    }

    public void setElevator(double sp){
        //SmartDashboard.putNumber("Elevator Setpoint", curSetPoint);
        elevator.elevatorToSetPoint(sp);
        // new ElevatorPID(elevator, sp);
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}
