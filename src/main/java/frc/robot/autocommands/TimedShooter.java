/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autocommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

//@author MARIA CRISTOFORO
public class TimedShooter extends CommandBase {

  private double timeToRun;

   private double endTime;

   private Timer timer = RobotContainer.shooterTimer;
  /**
   * Creates a new TimedShooter.
   */
  public TimedShooter(double t) {
    // Use addRequirements() here to declare subsystem dependencies.
    timeToRun = t;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double startTime = timer.get();
    endTime = startTime + timeToRun;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.shooterBase.ShootBallOn();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (timer.get() >= endTime);
  }
}
