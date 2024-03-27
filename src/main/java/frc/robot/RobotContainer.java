package frc.robot;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AutoCommands.AutoCommandHolder;
import frc.robot.commands.ScoreCommands.LiftSetpointDown;
import frc.robot.commands.ScoreCommands.LiftSetpointUp;
import frc.robot.commands.ScoreCommands.ScoreCommandHolder;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class RobotContainer {

  public static CommandXboxController driverController= new CommandXboxController(0); 
  public static CommandXboxController coDriverController= new CommandXboxController(1); 

  SendableChooser<Command> AutoChooser = new SendableChooser<>();

  //SUBSYSTEM
  EndEffectorSubsystem endEffectorSubsystem = new EndEffectorSubsystem();
  SwerveDriveSubsystem swerveDriveSubsystem = new SwerveDriveSubsystem();
  

  //COMMANDS
  ScoreCommandHolder scoreCommands = new ScoreCommandHolder(endEffectorSubsystem);
 
 



  //TRIGGERS 
  Trigger yButton = coDriverController.y(); 
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
    configureBindings();
     swerveDriveSubsystem.setDefaultCommand(
        new RunCommand(
            () -> swerveDriveSubsystem.drive(
                -MathUtil.applyDeadband(driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(driverController.getRightX(), OIConstants.kDriveDeadband),
                true, true),
            swerveDriveSubsystem));
    setAutoCommands();
    SmartDashboard.putData("Autos", AutoChooser);
  }

  private void configureBindings() {
    //Driver Controls
    aDriverButton.onTrue(scoreCommands.intakeNote());
    leftDriverTrigger.onTrue(scoreCommands.scoreAmp()); 
    rightDriverTrigger.onTrue(scoreCommands.scoreSpeaker());
    bDriverButton.onTrue(scoreCommands.shootNote());
    yDriverButton.onTrue(scoreCommands.compactPosition());
    xDriverButton.onTrue(scoreCommands.intakeDown());
    startButton.onTrue(scoreCommands.getHangReady());

 
    //Co-Driver Controls
    // xButton.onTrue(new RunCommand(() -> swerveDriveSubsystem.setX(), swerveDriveSubsystem));
    yButton.onTrue(new InstantCommand(swerveDriveSubsystem::zeroHeading));
    bButton.onTrue(scoreCommands.outtake());
    dpadDownCoDriver.onTrue(scoreCommands.hang());
    rightTriggerCoDriver.onTrue(scoreCommands.setFlyWheelZero());
    leftTriggerCoDriver.onTrue(scoreCommands.setFlyWheel());
    dpadRight.whileTrue(new LiftSetpointUp(endEffectorSubsystem));
    dpadleft.whileTrue(new LiftSetpointDown(endEffectorSubsystem));
   
    

  }

  public void setAutoCommands(){
    AutoCommandHolder autos = new AutoCommandHolder(endEffectorSubsystem, scoreCommands, swerveDriveSubsystem); 
    AutoChooser.addOption("DriveBack", autos.driveBack(3)); //anywhere
    AutoChooser.addOption("Speaker", autos.autoSpeaker()); //anywhere
    AutoChooser.addOption("SpeakerTaxi", autos.autoSpeakerTaxi()); //anywhere with caution--prefferably center
    AutoChooser.addOption("2-SpeakerTaxi", autos.autoCenterSpeakerTaxiIntakeSpeaker()); //center only
  }

  public Command getAutonomousCommand() {
    return AutoChooser.getSelected(); 
  }

}
