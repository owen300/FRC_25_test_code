package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.LimelightCommand;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class RobotContainer {

  CommandXboxController driverController= new CommandXboxController(0); 
  CommandXboxController coDriverController= new CommandXboxController(1); 

  SendableChooser<Command> AutoChooser = new SendableChooser<>();

  //Subsystem
  SwerveDriveSubsystem swerveDriveSubsystem = new SwerveDriveSubsystem();
  LimelightSubsystem limelightSubsystem = new LimelightSubsystem();

  //Commands
  LimelightCommand limelightCommand = new LimelightCommand(limelightSubsystem); 


  //Triggers 
  Trigger yButton = coDriverController.y(); 
  Trigger xButton = coDriverController.x(); 
  Trigger aButton = driverController.a(); 


  public RobotContainer() {
    configureBindings();
     swerveDriveSubsystem.setDefaultCommand(
        new RunCommand(
            () -> swerveDriveSubsystem.drive(
                -MathUtil.applyDeadband(driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(driverController.getRightX(), OIConstants.kDriveDeadband),
                true, true),
            swerveDriveSubsystem));
  }

  private void configureBindings() {

     //Driver Controls
    aButton.onTrue(limelightCommand);


    //Co-Driver Controls
    xButton.onTrue(new RunCommand(() -> swerveDriveSubsystem.setX(), swerveDriveSubsystem));
    yButton.onTrue(new InstantCommand(swerveDriveSubsystem::zeroHeading));


  }

}
