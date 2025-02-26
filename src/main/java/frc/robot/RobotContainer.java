// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import frc.robot.commands.ElevatorPID;
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

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    private double speedLimiter = 0.5;
    private double curSetPoint = 0.0;

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.02).withRotationalDeadband(MaxAngularRate * 0.02) // Add a 2% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandXboxController subjoystick = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final Elevator elevator = new Elevator();
    private final AlgaeRollers algaeroller = new AlgaeRollers();
    private final CoralRollers coralroller = new CoralRollers();
    private final Climb climb = new Climb();

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);

        // register commands to pathplanner
        // NamedCommands.registerCommand("elevatorL4", new ElevatorPID(elevator, ElevatorConstants.l4));
        // NamedCommands.registerCommand("elevatorL1", new ElevatorPID(elevator, ElevatorConstants.l1));
        NamedCommands.registerCommand("elevatorL4", elevator.elevatorToSetPoint(ElevatorConstants.l4));
        NamedCommands.registerCommand("elevatorL1", elevator.elevatorToSetPoint(ElevatorConstants.l1));
        NamedCommands.registerCommand("scoreOut", new RunCommand(() -> coralroller.scoreOut()));
        NamedCommands.registerCommand("preRoller", new RunCommand(() -> coralroller.preRoller()));
        NamedCommands.registerCommand("stopCoralRollers", new RunCommand(() -> coralroller.preRoller()));

        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(joystick.getLeftY() * MaxSpeed * speedLimiter) // Drive forward with negative Y (forward)
                    .withVelocityY(joystick.getLeftX() * MaxSpeed * speedLimiter) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate * speedLimiter) // Drive counterclockwise with negative X (left)
            )
        );

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

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
        // subjoystick.x().onTrue(new ElevatorPID(elevator, ElevatorConstants.l1));
        // subjoystick.y().onTrue(new ElevatorPID(elevator, ElevatorConstants.l2));
        // subjoystick.a().onTrue(new ElevatorPID(elevator, ElevatorConstants.l3));
        // subjoystick.b().onTrue(new ElevatorPID(elevator, ElevatorConstants.l4));

        subjoystick.x().onTrue(elevator.elevatorToSetPoint(ElevatorConstants.l1));
        subjoystick.y().onTrue(elevator.elevatorToSetPoint(ElevatorConstants.l2));
        subjoystick.a().onTrue(elevator.elevatorToSetPoint(ElevatorConstants.l3));
        subjoystick.b().onTrue(elevator.elevatorToSetPoint(ElevatorConstants.l4));
        subjoystick.povLeft().whileTrue(new RunCommand(() -> climb.pull()));
        subjoystick.povLeft().onFalse(new RunCommand(() -> climb.stop()));
        subjoystick.povRight().whileTrue(new RunCommand(() -> climb.reset()));
        subjoystick.povRight().onFalse(new RunCommand(() -> climb.stop()));

        //subjoystick.povUp().onTrue(elevator.disableElevatorPID());
        //subjoystick.povDown().onTrue(elevator.disableElevatorPID());
        subjoystick.povUp().whileTrue(new RunCommand(() -> elevator.elevatorToSetPoint(elevator.getSetPoint() + 0.3)));
        subjoystick.povDown().whileTrue(new RunCommand(() -> elevator.elevatorToSetPoint(elevator.getSetPoint() - 0.3)));

        //subjoystick.povUp().whileTrue(new RunCommand(() -> elevator.setMotor(ElevatorConstants.elevatorSpeed)));
        //subjoystick.povDown().whileTrue(new RunCommand(() -> elevator.setMotor(-ElevatorConstants.elevatorSpeed)));
        //subjoystick.povUp().onFalse(new RunCommand(() -> elevator.stopElevator()));
        //subjoystick.povDown().onFalse(new RunCommand(() -> elevator.stopElevator()));

        joystick.povUp().whileTrue(new RunCommand(() -> this.setSpeed(1.000)));
        joystick.povRight().whileTrue(new RunCommand(() -> this.setSpeed(0.500)));
        joystick.povLeft().whileTrue(new RunCommand(() -> this.setSpeed(0.200)));
        joystick.povDown().whileTrue(new RunCommand(() -> this.setSpeed(0.066)));
        
        //rollers for subsystems
        subjoystick.rightBumper().whileTrue(new RunCommand(() -> coralroller.scoreOut()));
        subjoystick.rightBumper().onFalse(new RunCommand(() -> coralroller.stopCoralRollers()));
        subjoystick.leftBumper().onFalse(new RunCommand(() -> coralroller.stopCoralRollers()));
        subjoystick.leftBumper().whileTrue(new RunCommand(() -> coralroller.preRoller()));

        subjoystick.rightBumper().whileTrue(new RunCommand(() -> algaeroller.spinOut()));
        subjoystick.rightBumper().onFalse(new RunCommand(() -> algaeroller.stopAlgaeRollers()));
        subjoystick.leftBumper().onFalse(new RunCommand(() -> algaeroller.stopAlgaeRollers()));
        subjoystick.leftBumper().whileTrue(new RunCommand(() -> algaeroller.spinIn()));
    }

    public void setSpeed(double spe){
        speedLimiter = spe;
        if(spe == 0.066) SmartDashboard.putString("Swerve Speed", "CRAWL");
        if(spe == 0.2) SmartDashboard.putString("Swerve Speed", "LOW");
        if(spe == 0.5) SmartDashboard.putString("Swerve Speed", "MID");
        if(spe == 1.0) SmartDashboard.putString("Swerve Speed", "HIGH");
    }

    public void IncSpeed(){
        if (speedLimiter < 1.0){
            speedLimiter += 0.1;
            SmartDashboard.putNumber("Swerve Speed", speedLimiter);
        }
    }

    public void DecSpeed(){
        if (speedLimiter > 0.1){
            speedLimiter -= 0.1;
            SmartDashboard.putNumber("Swerve Speed", speedLimiter);
        }
    }

    public void setElevator(double sp){
        curSetPoint = sp;
        SmartDashboard.putNumber("Elevator Setpoint", curSetPoint);
        elevator.elevatorToSetPoint(sp);
        // new ElevatorPID(elevator, sp);
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}
