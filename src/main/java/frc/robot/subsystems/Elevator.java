package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.ElevatorConstants;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
// import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Elevator extends SubsystemBase {
    //1 = left (facing forwards), 2 = right (facing forwards)
    SparkMax elevatorMotor1 = new SparkMax(ElevatorConstants.elevator1CanID, MotorType.kBrushless);
    SparkMax elevatorMotor2 = new SparkMax(ElevatorConstants.elevator2CanID, MotorType.kBrushless);
    SparkMaxConfig elevatorMotor1Config = new SparkMaxConfig();
    SparkMaxConfig elevatorMotor2Config = new SparkMaxConfig();
    RelativeEncoder elevatorEncoder1 = elevatorMotor1.getEncoder();
    RelativeEncoder elevatorEncoder2 = elevatorMotor2.getEncoder();
    PIDController elevatorPID;
    boolean canUp = true;
    boolean canDown = true;
    double status = 1.0;
    boolean locked = true;

    public Elevator() {
        elevatorMotor1Config
            .inverted(true)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(40);

        elevatorEncoder1.setPosition(0);
        elevatorEncoder2.setPosition(0);
        //elevatorEncoder.setInverted(true);

        elevatorMotor2Config
            .inverted(true)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(40);

        elevatorMotor1.configure(elevatorMotor1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        elevatorMotor2.configure(elevatorMotor2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        elevatorPID = new PIDController(ElevatorConstants.eP, ElevatorConstants.eI, ElevatorConstants.eD);
        elevatorPID.setTolerance(1);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator position", (elevatorEncoder1.getPosition()  + elevatorEncoder2.getPosition()) / 2.0);
        SmartDashboard.putNumber("Elevator effective power", ((elevatorMotor1.getAppliedOutput() + elevatorMotor2.getAppliedOutput()) / 2.0));
    }

    public void setMotor(double speed){
        elevatorMotor1.set(speed);
        elevatorMotor2.set(speed);
    }

    public double sign(double x){
        if(x < 0) return -1;
        else return 1;
    }

    public void raiseElevator() {
        elevatorMotor1.set(ElevatorConstants.elevatorSpeed);
        elevatorMotor2.set(ElevatorConstants.elevatorSpeed);
    }

    public void lowerElevator(){
        elevatorMotor1.set(-ElevatorConstants.elevatorSpeed);
        elevatorMotor2.set(-ElevatorConstants.elevatorSpeed);
    }
    
    public void stopElevator(){
        elevatorMotor1.set(0);
        elevatorMotor2.set(0);
    }

    public double getEncoderPos() {
        return ((elevatorEncoder1.getPosition() + elevatorEncoder2.getPosition()) / 2.0);
    }

    public Command elevatorToSetPoint(double elevatorSetPoint){
        elevatorPID.setSetpoint(elevatorSetPoint);
        SmartDashboard.putNumber("Elevator setpoint", elevatorSetPoint);
        return run(() -> setMotor(elevatorPID.calculate(getEncoderPos(), elevatorSetPoint)));
    }

    public Command disableElevatorPID(){
        return run(() -> elevatorPID.reset());
    }

    public void unlock(){
        locked = false;
        //elevatorEncoder.setPosition(1);
    }

    public void lock(){

    }

    public void score(){

    }

    public void stopAll(){

    }
}