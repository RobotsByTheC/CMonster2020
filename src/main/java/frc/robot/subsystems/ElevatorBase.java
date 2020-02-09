/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

//@author ELLA WARNOCK, XANDER EVERITT 
public class ElevatorBase extends SubsystemBase {

  public static Spark ElevatorMotor = RobotContainer.elevatorSpark;
  public static Spark HookMotor = RobotContainer.hookSpark;
  /**
   * Creates a new ClimberBase.
   */
  public ElevatorBase() {

  }
  public void ElevatorUp(){
    ElevatorMotor.set(1);
  }
  
  public void ElevatorDown(){
    ElevatorMotor.set(-1);
  }
  
  public void ElevatorStop(){
    ElevatorMotor.set(0);
  }

  public void HookUp(){
    HookMotor.set(1);
  }
  
  public void HookDown(){
    HookMotor.set(-1);
  }
  
  public void HookStop(){
    HookMotor.set(0);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
