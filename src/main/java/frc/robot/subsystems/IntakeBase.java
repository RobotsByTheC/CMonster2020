/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.Spark;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;

//@author ELLA WARNOCK
public class IntakeBase extends SubsystemBase {

  public static WPI_TalonSRX intakeTalon = RobotContainer.intakeTalon; 
  public static DoubleSolenoid IntakeSolenoid = RobotContainer.intakeSolenoid; 
  /**
   * Creates a new IntakeBase.
   */
  public IntakeBase() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void intakeStart(){
    intakeTalon.set(0.75);
  }

  public void intakeStartOut(){
    intakeTalon.set(-0.75);
  }

  public void intakeStop(){
    intakeTalon.set(0);
  }

  public void intakeSolenoidUp(){
    IntakeSolenoid.set(DoubleSolenoid.Value.kForward);
  }

  public void intakeSolenoidDown(){
    IntakeSolenoid.set(DoubleSolenoid.Value.kReverse);
  }
}
