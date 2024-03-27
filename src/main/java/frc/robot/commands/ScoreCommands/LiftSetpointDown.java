package frc.robot.commands.ScoreCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffectorSubsystem;


public class LiftSetpointDown extends Command{

    private final EndEffectorSubsystem liftSubsystem;
    private double setPoint;
    double startTime; 

    public LiftSetpointDown(EndEffectorSubsystem liftSubsystem){
        this.liftSubsystem = liftSubsystem; 
        addRequirements(liftSubsystem);
    }

    @Override 
    public void initialize(){
       startTime = Timer.getFPGATimestamp();
       setPoint = liftSubsystem.getPose();
    }

    @Override 
    public void execute(){
           setPoint +=0.01; 
    }

    @Override 
    public boolean isFinished(){ 
      return Timer.getFPGATimestamp() - startTime > 0.5;
    }

    public void end(boolean interuppted){
        EndEffectorSubsystem.lift(setPoint);   
    }
    
}