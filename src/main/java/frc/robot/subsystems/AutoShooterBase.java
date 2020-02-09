/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import edu.wpi.first.networktables.NetworkTableInstance;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;


//@author MARIA CRISTOFORO
public class AutoShooterBase extends SubsystemBase {


  public WPI_TalonSRX rightTalon = RobotContainer.rightTalon;
  public WPI_VictorSPX rightVictor = RobotContainer.rightVictor;
  public WPI_TalonSRX leftTalon = RobotContainer.leftTalon;
  public WPI_VictorSPX leftVictor = RobotContainer.leftVictor;

  double leftMotorSpeed = 0;
  double rightMotorSpeed = 0;

  double KpAim = -0.1f;
  double KpDistance = -0.1f;
  double min_command = 0.05f;


  double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
  double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);

  double heading_error = -tx;
  double distance_error = -ty;
  double steering_adjust = 0.0f;
  //these 3 need to be tuned 

  double desired_distance = 10.0; //needs to be changed after testing
       //need to tune these - not to high or else oscillation

  /**
   * Creates a new AutoShooterBase.
   */
  public AutoShooterBase() {

  }

  public void adjust(){
    //adjusting calculations, taken from driveBase
    double current_distance = RobotContainer.driveBase.EstimateDistance();
      
    if (tx > 1.0){
      steering_adjust = KpAim*heading_error - min_command;
    }

    else if (tx < 1.0){
      steering_adjust = KpAim*heading_error + min_command;
    }

    double distance_adjust = KpDistance * distance_error;

    leftMotorSpeed += steering_adjust + distance_adjust;
    rightMotorSpeed -= steering_adjust + distance_adjust;

  

    leftTalon.set(leftMotorSpeed); // finally assign the stored values to talons  
    leftVictor.set(leftMotorSpeed);
    rightTalon.set(rightMotorSpeed);
    rightVictor.set(rightMotorSpeed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
