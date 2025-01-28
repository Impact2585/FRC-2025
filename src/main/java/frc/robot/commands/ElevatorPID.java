package frc.robot.commands;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants.ElevatorConstants;

import frc.robot.subsystems.Elevator;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.CommandBase;

import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.wpilibj.smartdashboard.*;

public class ElevatorPID extends Command {
    private Elevator elevatorSubsystem;
    private PIDController elevatorPID;

    public ElevatorPID(Elevator elevatorSubsystem, double setpoint) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.elevatorPID = new PIDController(//
                ElevatorConstants.eP, ElevatorConstants.eI, ElevatorConstants.eD);
        this.elevatorPID.setTolerance(1);
        elevatorPID.setSetpoint(setpoint);

        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("Elevator PID started!");
        elevatorPID.reset();
    }

    @Override
    public void execute() {
        double speed = elevatorPID.calculate(elevatorSubsystem.getEncoderPos(), elevatorPID.getSetpoint());
        elevatorSubsystem.setMotor(speed);
        SmartDashboard.putNumber("Elevator setpoint", elevatorPID.getSetpoint());
    }

    @Override
    public void end(boolean interrupted) {
        elevatorSubsystem.stopAll();
        System.out.println("Elevator PID ended!");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}