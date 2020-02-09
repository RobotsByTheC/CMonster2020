/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import frc.robot.*;

//@author MARIA CRISTOFORO
public class LimelightBase extends SubsystemBase {
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

  public WPI_TalonSRX rightTalon = RobotContainer.rightTalon;
  public WPI_VictorSPX rightVictor = RobotContainer.rightVictor;
  public WPI_TalonSRX leftTalon = RobotContainer.leftTalon;
  public WPI_VictorSPX leftVictor = RobotContainer.leftVictor;

  double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
  double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
  double ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);

  double rightMotorSpeed = 0;
  double leftMotorSpeed = 0;

  double KpAim = -0.1f;
  
  double min_command = 0.05f;


  double KpDistance = 0.3f;

  double current_distance = RobotContainer.driveBase.EstimateDistance();
  double desired_distance = 10.0; //needs to be changed after testing
   //need to tune these - not to high or else oscillation


 
  /**
   * Creates a new LimelightBase.
   */
  public LimelightBase() {
    //NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);
    //set pipeline - doesn't seem to be necessary unless there is more than 1 pipeline 
    
    

  }

  public void AngleControl(){
    SmartDashboard.putBoolean("LogitechButton3", RobotContainer.getLogitech().getRawButton(3));
        
    double heading_error = -tx;
    double steering_adjust = 0.0f;

      if (tx > 1.0f){
        steering_adjust = KpAim * heading_error - min_command;
      }

      else if (tx < 1.0f){
        steering_adjust = KpAim * heading_error + min_command;
      }

      leftMotorSpeed += steering_adjust;
      rightMotorSpeed -= steering_adjust;

      leftTalon.set(leftMotorSpeed); // finally assign the stored values to talons  
      leftVictor.set(leftMotorSpeed);
      rightTalon.set(rightMotorSpeed);
      rightVictor.set(rightMotorSpeed);

  }

  public void DistanceControl(){

      SmartDashboard.putBoolean("LogitechButton2", RobotContainer.getLogitech().getRawButton(2));
        

      double distance_error = desired_distance - current_distance;
      SmartDashboard.putNumber("DistanceError", distance_error);
      double driving_adjust = KpDistance * distance_error;
      SmartDashboard.putNumber("DrivingAdjust", driving_adjust);

      leftMotorSpeed += driving_adjust;
      rightMotorSpeed += driving_adjust;

      leftTalon.set(leftMotorSpeed); // finally assign the stored values to talons  
      leftVictor.set(leftMotorSpeed);
      rightTalon.set(rightMotorSpeed);
      rightVictor.set(rightMotorSpeed);

  }

  public void DistanceAngle() {
    double heading_error = -tx;
    double distance_error = -ty;
    double steering_adjust = 0.0f;

    if (tx > 1.0)
    {
            steering_adjust = KpAim*heading_error - min_command;
    }
    else if (tx < 1.0)
    {
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
    //where you should put values that continuously update

    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");

    //read values periodically
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);

    //post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);
  }
}
