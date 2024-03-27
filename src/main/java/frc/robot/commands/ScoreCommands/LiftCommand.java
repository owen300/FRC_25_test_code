package frc.robot.commands.ScoreCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffectorSubsystem;


public class LiftCommand extends Command{

    private final double setPoint;
    private final double TOLERANCE = 0.5; 
    double startTime;

    public LiftCommand(double setpoint){
        this.setPoint = setpoint;
        addRequirements();
    }

    @Override 
    public void initialize(){
        startTime = Timer.getFPGATimestamp();
    }

    @Override 
    public void execute(){
       EndEffectorSubsystem.lift(setPoint);       
    }

    @Override 
    public boolean isFinished(){
       return (EndEffectorSubsystem.getLiftDistance() < setPoint + TOLERANCE && EndEffectorSubsystem.getLiftDistance() > setPoint - TOLERANCE) && (Timer.getFPGATimestamp() - startTime > 0.5);
    }
    
}