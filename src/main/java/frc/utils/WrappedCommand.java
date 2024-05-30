package frc.utils;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.State;
import frc.robot.State.robotState;

public class WrappedCommand extends Command{//depreciated, dont plan on using
    @Override
    public void end(boolean interrupted){
        State.setState(robotState.Stable);
    }
}
