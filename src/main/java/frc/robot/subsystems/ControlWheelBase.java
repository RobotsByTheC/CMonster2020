/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.DoubleSolenoid;

//@author ELLA WARNOCK
public class ControlWheelBase extends SubsystemBase {

 public static WPI_TalonSRX ControlWheel = RobotContainer.controlWheel; 
 public static DoubleSolenoid ControlWheelSolenoid = RobotContainer.controlWheelSolenoid; 
  /**
   * Creates a new ControlWheelBase.
   */
  public ControlWheelBase() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void wheelStart(){
    ControlWheel.set(0.75);
  }

  public void wheelStop(){
    ControlWheel.set(0);
  }
  public void controlWheelSolenoidUp(){
    ControlWheelSolenoid.set(DoubleSolenoid.Value.kForward);
  }

  public void controlWheelSolenoidDown(){
    ControlWheelSolenoid.set(DoubleSolenoid.Value.kReverse);
  }
}
