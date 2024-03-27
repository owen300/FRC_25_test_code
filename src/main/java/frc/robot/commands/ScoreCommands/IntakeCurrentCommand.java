package frc.robot.commands.ScoreCommands;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffectorSubsystem;
import static frc.robot.Constants.SubsystemConstants;

public class IntakeCurrentCommand extends Command{

    private final EndEffectorSubsystem intakeSubsystem;
    private final double speed;
    private final boolean inverted; 
    private Debouncer filter_debouncer = new Debouncer(SubsystemConstants.kIntakeDebounceTime, Debouncer.DebounceType.kBoth);

    public IntakeCurrentCommand(EndEffectorSubsystem intakeSubsystem, double speed, boolean inverted){
        this.intakeSubsystem = intakeSubsystem; 
        this.speed = speed;
        this.inverted = inverted; 
        addRequirements(intakeSubsystem);
    }

    @Override 
    public void initialize(){
      intakeSubsystem.setIntakeSpeedDirection(speed, inverted);
    }

    @Override 
    public boolean isFinished(){ 
        if(SubsystemConstants.kIntakeDebounce) { 
            return filter_debouncer.calculate(EndEffectorSubsystem.filteredCurrentIntake>SubsystemConstants.kIntakeCurrentThreshold);
        } else {
            return EndEffectorSubsystem.filteredCurrentIntake > SubsystemConstants.kIntakeCurrentThreshold;
        } 
    }

    @Override
    public void end(boolean interrupted){
      if(interrupted){
            intakeSubsystem.setIntakeSpeedDirection(0, inverted); 
        }
        else {
            intakeSubsystem.setIntakeSpeedDirection(0, inverted);
        }
    }
}