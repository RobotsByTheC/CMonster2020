/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.*;
import frc.robot.Robot;

//@author MARIA CRISTOFORO, ELLA WARNOCK
public class DriveWithJoystick extends CommandBase {

  private final DriveBase driveBase = RobotContainer.driveBase;
 
  /**
   * Creates a new DriveWithJoystick.
   */
  public DriveWithJoystick() {
    addRequirements(driveBase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   
    RobotContainer.driveBase.JoystickInputs(RobotContainer.getRightJoystick(), RobotContainer.getLeftJoystick(), RobotContainer.getLogitech());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
