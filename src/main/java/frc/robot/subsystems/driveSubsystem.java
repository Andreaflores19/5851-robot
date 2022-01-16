// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.util.WPILibVersion;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class driveSubsystem extends SubsystemBase {
  WPI_TalonSRX leftFront= new WPI_TalonSRX(12);
  WPI_TalonSRX rightRear= new WPI_TalonSRX(11);
  WPI_VictorSPX rightFront= new WPI_VictorSPX(18);
  Spark leftRear= new Spark(1);

  SpeedControllerGroup rightGroup = new SpeedControllerGroup(rightFront, rightRear);
  SpeedControllerGroup leftGroup = new SpeedControllerGroup(leftFront, leftRear);

  DifferentialDrive drive = new DifferentialDrive(leftGroup, rightGroup);

  boolean orientation= false;

double rightEncoderPosition;
double leftEncoderPosition;

double KPTF= (6/49152)*Math.PI;


public void flipMotor(){
  orientation = !orientation;

  rightGroup.setInverted(orientation);
  leftGroup.setInverted(orientation);
}

public void moveMotors(double leftSpeed, double rightSpeed){

  if(orientation){ drive.tankDrive(rightSpeed, leftSpeed);
  }
  else{drive.tankDrive(leftSpeed, rightSpeed);}
}

public void resetEncoder(){

  rightRear.setSelectedSensorPosition(0);
  leftFront.setSelectedSensorPosition(0);
  
  rightRear.setSensorPhase(true);
}
  /** Creates a new driveSubsystem. */
  public driveSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    leftEncoderPosition= leftFront.getSelectedSensorPosition();
    rightEncoderPosition= rightRear.getSelectedSensorPosition();
  }

  public double leftEncoder() {
    return leftEncoderPosition *KPTF;

  }

  public double rightEncoder(){

    return rightEncoderPosition *KPTF;
  }
}
