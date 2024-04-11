package frc.robot;


import java.io.File;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class RobotContainer {

  public static CommandXboxController driverController= new CommandXboxController(0); 
  public static CommandXboxController coDriverController= new CommandXboxController(1); 

  SendableChooser<Command> AutoChooser = new SendableChooser<>();

  //SUBSYSTEM
  //SmartSwerveDriveSubsystem swerveDriveSubsystem = new SmartSwerveDriveSubsystem(limelightSubsystem);
  public SwerveDriveSubsystem swerveDriveSubsystem = new SwerveDriveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         "swerve"));
  

 
 



  //TRIGGERS 
  Trigger yButton = driverController.y(); 
  Trigger xButton = coDriverController.x(); 
  Trigger aButton = coDriverController.a();
  Trigger bButton = coDriverController.b();
  Trigger dpadUpCoDriver = coDriverController.povUp();
  Trigger dpadDownCoDriver = coDriverController.povDown();
  Trigger leftBumperCoDriver = coDriverController.leftBumper(); 
  Trigger rightBumperCoDriver = coDriverController.rightBumper(); 
  Trigger rightTriggerCoDriver = coDriverController.rightTrigger(); 
  Trigger leftTriggerCoDriver = coDriverController.leftTrigger(); 
  Trigger dpadRight = coDriverController.povRight();
  Trigger dpadleft = coDriverController.povLeft();


  Trigger aDriverButton = driverController.a(); 
  Trigger bDriverButton = driverController.b(); 
  Trigger xDriverButton = driverController.x(); 
  Trigger yDriverButton = driverController.y(); 
  Trigger leftDriverTrigger = driverController.leftTrigger(); 
  Trigger leftDriverBumper = driverController.leftBumper(); 
  Trigger rightDriverTrigger = driverController.rightTrigger();
  Trigger rightDriverBumper = driverController.rightBumper();
  Trigger dpadUpDriver = driverController.povUp();
  Trigger dpadDownDriver = driverController.povDown();
  Trigger startButton = driverController.start();


 


  public RobotContainer() {
    registerNamedCommands();
    configureBindings();
     swerveDriveSubsystem.setDefaultCommand(
        new RunCommand(
            () -> swerveDriveSubsystem.driveCommand(
              () -> MathUtil.applyDeadband(driverController.getLeftY(), Constants.OIConstants.kDriveDeadband),
              () -> MathUtil.applyDeadband(driverController.getLeftX(), Constants.OIConstants.kDriveDeadband),
              () -> driverController.getRightX() * 0.5),
            swerveDriveSubsystem));
    setAutoCommands();
   // SmartDashboard.putData("Autos", AutoChooser);
  }

  private void configureBindings() {
    //Driver Controls
   
 
    //Co-Driver Controls
    // xButton.onTrue(new RunCommand(() -> swerveDriveSubsystem.setX(), swerveDriveSubsystem));
    yButton.onTrue(new InstantCommand(swerveDriveSubsystem::zeroGyro));
    

  }

  public void registerNamedCommands() {

     }

  public void setAutoCommands(){
   }

  public Command getAutonomousCommand() {
    return AutoChooser.getSelected(); 
  }

}
