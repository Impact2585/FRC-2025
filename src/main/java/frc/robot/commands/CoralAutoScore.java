package frc.robot.commands;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Constants.ElevatorConstants;

import frc.robot.subsystems.CoralRollers;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj2.command.CommandBase;

import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.wpilibj.smartdashboard.*;

public class CoralAutoScore extends Command {
    private CoralRollers coralRollers;
    private PIDController elevatorPID;
    private Timer endTimer;

    public CoralAutoScore(CoralRollers coralRollers) {
        this.coralRollers = coralRollers;
        addRequirements(coralRollers);
    }

    @Override
    public void initialize() {
        this.endTimer = new Timer();
        this.endTimer.start();
    }

    @Override
    public void execute() {
        coralRollers.preRoller();
    }

    @Override
    public void end(boolean interrupted) {
    
    }

    @Override
    public boolean isFinished() {
        return endTimer.hasElapsed(1);
    }
}