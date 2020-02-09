/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autocommands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveBase;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html

//@author MARIA CRISTOFORO
public class MoveForward extends PIDCommand { 
  /**
   * Creates a new MoveForward.
   */
  public MoveForward(double setpoint, DriveBase driveBase) {
    super(
        // The controller that the command will use
        new PIDController(RobotContainer.kMoveP, RobotContainer.kMoveP, RobotContainer.kMoveD),
        // This should return the measurement
        () -> 0,
        // This should return the setpoint (can also be a constant)
        setpoint,
        // This uses the output
        output -> {
          // Use the output here
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.


    //HALT PROGRESS FOR NOW BEFORE FINDING OUT WHEHTER PIDS ARE STRICTLY NECESSARY
    //FOR OUR AUTO STRATEGY

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
