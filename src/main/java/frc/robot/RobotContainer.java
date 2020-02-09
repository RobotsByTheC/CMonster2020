/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj2.command.CommandScheduler;


/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */


//@author MARIA CRISTOFORO, ELLA WARNOCK, ALEXANDER WOLF, XANDER EVERITT 
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  
  public static DriveBase driveBase;
  public static LimelightBase limelightBase;
  public static AutoShooterBase autoShooterBase;

  public static ShooterBase shooterBase;
  public static ElevatorBase elevatorBase;
  public static ControlWheelBase controlWheelBase; 
  public static IntakeBase intakeBase; 

 
  public static MoveForward moveForward;
  public static DriveWithJoystick driveWithJoystick;
  
  //Talons 
  public static WPI_TalonSRX rightTalon = new WPI_TalonSRX(2);
  public static WPI_VictorSPX rightVictor = new WPI_VictorSPX(3);
  public static WPI_TalonSRX leftTalon = new WPI_TalonSRX(4);
  public static WPI_VictorSPX leftVictor = new WPI_VictorSPX(5);
  public static WPI_TalonSRX controlWheel = new WPI_TalonSRX (6);
  public static WPI_TalonSRX conveyorTalon = new WPI_TalonSRX(7);
  public static WPI_TalonSRX intakeTalon = new WPI_TalonSRX(8);
  
  //Sparks
  public static Spark elevatorSpark = new Spark(0);
  public static Spark hookSpark = new Spark (1); 
  public static Spark shooterLeftSpark = new Spark(2);
  public static Spark shooterRightSpark = new Spark(3);
 
  
  //Solenoids
  public static DoubleSolenoid intakeSolenoid = new DoubleSolenoid (0,1);
  public static DoubleSolenoid controlWheelSolenoid = new DoubleSolenoid (2,3);

  public static Compressor robotCompressor;


  //Joysticks and Joystick Buttons 
  public static Joystick rightJoystick; 
  public static Joystick leftJoystick;
  public static Joystick logitech;

  public static JoystickButton visionAngle;
  public static JoystickButton visionDistance;
  public static JoystickButton visionAngleDistance;
  public static JoystickButton quarterSpeed;

  public static JoystickButton elevatorUp;
  public static JoystickButton elevatorDown;

  public static JoystickButton intakeButtonIn;
  public static JoystickButton intakeButtonOut;
  public static JoystickButton intakeUp;
  public static JoystickButton intakeDown; 

  public static JoystickButton controlWheelUp; 
  public static JoystickButton controlWheelDown;
  public static JoystickButton wheelControl;

  public static JoystickButton shootBall;
  public static JoystickButton conveyorBeltUp;
  public static JoystickButton conveyorBeltDown;
  public static JoystickButton hookUp;
  public static JoystickButton hookDown;
  


  //Vision and Auto
  public static double kMoveP;
  public static double kMoveI;
  public static double kMoveD;

  public static Timer moverTimer;
  public static Timer shooterTimer;


  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    //initialize joysticks 
    rightJoystick = new Joystick(0);
    leftJoystick = new Joystick(1);
    logitech = new Joystick(2);

    robotCompressor = new Compressor(0);
    robotCompressor.setClosedLoopControl(true);



    //initialize buttons:

    //vision buttons
    //visionAngle = new JoystickButton(logitech, 3);
    //visionDistance = new JoystickButton(logitech, 2);

    //buttons for driver:
    intakeUp = new JoystickButton(rightJoystick, 2);
    intakeDown = new JoystickButton(rightJoystick, 3);
    quarterSpeed = new JoystickButton(rightJoystick, 1);

    controlWheelUp = new JoystickButton(leftJoystick, 2);
    controlWheelDown = new JoystickButton(leftJoystick, 3);
    wheelControl = new JoystickButton (leftJoystick, 1);
   

    //buttons for operator
    intakeButtonIn = new JoystickButton(logitech, 2);
    intakeButtonOut = new JoystickButton(logitech, 3);

    visionAngleDistance = new JoystickButton(logitech, 1);
    shootBall = new JoystickButton(logitech, 4);
    conveyorBeltUp = new JoystickButton(logitech, 6);
    conveyorBeltDown = new JoystickButton(logitech, 8);

    elevatorUp = new JoystickButton(logitech, 5);
    elevatorDown = new JoystickButton(logitech, 7);
    hookUp = new JoystickButton(logitech, 10);
    hookDown = new JoystickButton(logitech, 9);



    //initialize and link drive subsystem and command  
    driveBase = new DriveBase();
    driveWithJoystick = new DriveWithJoystick();
    CommandScheduler.getInstance().setDefaultCommand(driveBase, driveWithJoystick);

    //initialize other subsystems 
    limelightBase = new LimelightBase();
    autoShooterBase = new AutoShooterBase();
    shooterBase = new ShooterBase();
    elevatorBase = new ElevatorBase();
    controlWheelBase = new ControlWheelBase();
    intakeBase = new IntakeBase();

    
    //inialize auto and vision constants and buttons 
    kMoveD = 0.05;
    kMoveI = 0.0;
    kMoveD = 0.0;
    moveForward = new MoveForward();
    moverTimer = new Timer();
    shooterTimer = new Timer();


    //now start linking buttons to commands:

    //commented out for now - want to use one vision button for final design
    //visionAngle.whileHeld(new VisionAngleControl());
    //visionAngle.whenReleased(new VisionControlStop());
    //visionDistance.whileHeld(new VisionDistanceControl());
    //visionDistance.whenReleased(new VisionControlStop());

    visionAngleDistance.whileHeld(new VisionAngleDistance());
    visionAngleDistance.whenReleased(new VisionControlStop());


    //make quarter speed command link here

    //intake subsystem
    intakeButtonIn.whileHeld(new IntakeStart());
    intakeButtonIn.whenReleased(new IntakeStop());
    intakeButtonOut.whileHeld(new IntakeStartOut());
    intakeButtonOut.whenReleased(new IntakeStop());
    intakeUp.whenPressed(new IntakeSolenoidUp()); 
    intakeDown.whenPressed(new IntakeSolenoidDown()); 

    //control wheel subsystem
    controlWheelUp.whenPressed(new ControlWheelUp());
    controlWheelDown.whenPressed(new ControlWheelDown());
    wheelControl.whileHeld(new WheelStart());
    wheelControl.whenReleased(new WheelStop());
    
    //shooter subsystem
    shootBall.whileHeld(new ShootBall());
    shootBall.whenReleased(new StopBall());
    conveyorBeltUp.whileHeld(new ConveyorStart());
    conveyorBeltUp.whenReleased(new ConveyorStop());
    conveyorBeltDown.whileHeld(new ConveyorStartDown());
    conveyorBeltDown.whenReleased(new ConveyorStop());

    //elevator subsystem
    elevatorUp.whileHeld(new ElevatorUpStart());
    elevatorUp.whenReleased(new ElevatorStop());
    elevatorDown.whileHeld(new ElevatorDownStart());
    elevatorDown.whenReleased(new ElevatorStop());
    hookUp.whileHeld(new WinchStartUp());
    hookUp.whenReleased(new WinchStop());
    hookDown.whileHeld(new WinchStartDown());
    hookDown.whenReleased(new WinchStop());

    // Configure the button bindings
    configureButtonBindings();
    
  }

  public static Joystick getRightJoystick(){
    return rightJoystick;
  }

  public static Joystick getLeftJoystick(){
    return leftJoystick;
  }

  public static Joystick getLogitech(){
    return logitech;
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return Robot.autoChooser.getSelected();
    //just do moveForward for now, but eventually will be the chosen autonomous command 

  }
}
