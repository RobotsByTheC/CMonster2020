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
import edu.wpi.first.wpilibj.Spark;


//@author ELLA WARNOCK, ALEXANDER WOLF
public class ShooterBase extends SubsystemBase {

//public static WPI_TalonSRX TiltShooter = RobotContainer.tiltShooter;
public static Spark leftSpark = RobotContainer.shooterLeftSpark;
public static Spark rightSpark = RobotContainer.shooterRightSpark;
public static WPI_TalonSRX conveyorTalon = RobotContainer.conveyorTalon;
  /**
   * Creates a new ShooterBase.
   */
  public ShooterBase() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


  

  public void ShootBallOn(){
    leftSpark.set(1);
    rightSpark.set(1);
  }
  public void ShootBallOff(){
    leftSpark.set(0);
    rightSpark.set(0);
  }

  public void ConveyorStart(){
    conveyorTalon.set(1);
  }
  public void ConveyorStop(){
    conveyorTalon.set(0);
  }
  public void ConveyorStartDown(){
    conveyorTalon.set(-1);
  }
}
