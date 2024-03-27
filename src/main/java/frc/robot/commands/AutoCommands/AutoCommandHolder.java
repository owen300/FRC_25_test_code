package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ScoreCommandHolderConstants;
import frc.robot.commands.ScoreCommands.IntakeCurrentCommand;
import frc.robot.commands.ScoreCommands.IntakeTimeCommand;
import frc.robot.commands.ScoreCommands.LiftCommand;
import frc.robot.commands.ScoreCommands.ScoreCommandHolder;
import frc.robot.commands.ScoreCommands.ShootCommand;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class AutoCommandHolder extends Command{
    
    EndEffectorSubsystem endEffectorSubsystem;
    ScoreCommandHolder scoreCommandHolder;
    static SwerveDriveSubsystem swerveDriveSubsystem; 


    public AutoCommandHolder(EndEffectorSubsystem endEffectorSubsystem, ScoreCommandHolder scoreCommandHolder, SwerveDriveSubsystem swerveDriveSubsystem){
        this.endEffectorSubsystem = endEffectorSubsystem; 
        this.scoreCommandHolder = scoreCommandHolder;   
        AutoCommandHolder.swerveDriveSubsystem = swerveDriveSubsystem; 
    }
    
    public Command driveBack(double time){
       return new DriveCommand(swerveDriveSubsystem, time);
    }

    public SequentialCommandGroup autoSpeaker(){
        return new SequentialCommandGroup(scoreSpeakerAuto(), new WaitCommand(3), shootNoteAuto());
    }

    public SequentialCommandGroup autoSpeakerTaxi(){
        return new SequentialCommandGroup(scoreSpeakerAuto(), new WaitCommand(3), shootNoteAuto(), driveBack(3));
    }

    public SequentialCommandGroup autoCenterSpeakerTaxiIntakeSpeaker(){
        return new SequentialCommandGroup(autoSpeakerTaxi(), intakeNoteAuto(), autoSpeaker() );
    }




    //METHODS
    public SequentialCommandGroup scoreSpeakerAuto(){
        return new SequentialCommandGroup(
            new ShootCommand(0.85),
            new LiftCommand(ScoreCommandHolderConstants.kSpeakerDistanceSetpoint),
            new LiftCommand(ScoreCommandHolderConstants.kSpeakerSetpoint)
        ); 
    }

     public SequentialCommandGroup shootNoteAuto(){
        return new SequentialCommandGroup(
            new IntakeTimeCommand(endEffectorSubsystem, 0.7, false, 2),
            new ShootCommand(0),
            compactPositionAuto()
        ); 
    }

    public SequentialCommandGroup intakeNoteAuto(){
        return new SequentialCommandGroup(
            new LiftCommand(ScoreCommandHolderConstants.kIntakeFirstSetpoint),
            new LiftCommand(ScoreCommandHolderConstants.kIntakeSecondSetpoint),
            new IntakeCurrentCommand(endEffectorSubsystem, 0.8, false), 
            new IntakeTimeCommand(endEffectorSubsystem, 0.1, true, 0.3),
            new LiftCommand (ScoreCommandHolderConstants.kCompactSetpoint)
        ); 
    }

    public SequentialCommandGroup compactPositionAuto(){
        return new SequentialCommandGroup(
            new LiftCommand(ScoreCommandHolderConstants.kCompactSetpoint),
            new IntakeCurrentCommand(endEffectorSubsystem, 0, false ),
            new ShootCommand(0) 
        ); 
    }





}