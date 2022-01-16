// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.sql.Time;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.driveSubsystem;

public class driveCommand extends CommandBase {

  driveSubsystem driveSub;
double currentD;
double desiredD;
double error;
double konstant;
double proportional;
double integral;
double pastTime;
double lastError;
  /** Creates a new driveCommand. */
  public driveCommand(driveSubsystem driveS, double desiredDistance) {
    desiredD= desiredDistance;
  driveSub= driveS;
   addRequirements(driveSub) ;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    lastError= 0;
    pastTime= Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
currentD= driveSub.leftEncoder();
error= desiredD- currentD;
proportional= Constants.kp*error;
double dt= Timer.getFPGATimestamp()-pastTime;
integral += (error*dt)* Constants.KI;
double derivative= Constants.kD* ((error-lastError)/dt);
double speed= proportional+integral+ derivative;
driveSub.moveMotors(speed, speed);
pastTime= Timer.getFPGATimestamp();

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
