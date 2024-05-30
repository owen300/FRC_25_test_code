package frc.robot.commands.oldordrivecommands.AutoCommands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDriveSubsystem;


public class DriveCommand extends Command{

    private final SwerveDriveSubsystem swerveDriveSubsystem;
    private double time; 
    double startTime; 
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(1, 0,0); 
    

    public DriveCommand(SwerveDriveSubsystem swerveDriveSubsystem, double time){
       this.swerveDriveSubsystem = swerveDriveSubsystem;
       this.time = time; 
        addRequirements(swerveDriveSubsystem);
    }

    @Override 
    public void initialize(){
        startTime = Timer.getFPGATimestamp();
      swerveDriveSubsystem.drive(chassisSpeeds);
    }

    @Override 
    public boolean isFinished(){ 
        return Timer.getFPGATimestamp() - startTime > time;
    }

    @Override
    public void end(boolean interrupted){}
}