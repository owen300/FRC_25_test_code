package frc.robot.commands.ScoreCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffectorSubsystem;


public class LiftSetpointUp extends Command{

    private final EndEffectorSubsystem liftSubsystem;
    private double setPoint;

    public LiftSetpointUp(EndEffectorSubsystem liftSubsystem){
        this.liftSubsystem = liftSubsystem; 
        addRequirements(liftSubsystem);
    }

    @Override 
    public void initialize(){
       setPoint = liftSubsystem.getPose();
    }

    @Override 
    public void execute(){
           setPoint -=0.01; 
    }

    public void end(boolean interuppted){
        EndEffectorSubsystem.lift(setPoint);   
    }
    
}