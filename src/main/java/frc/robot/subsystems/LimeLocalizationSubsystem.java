package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.utils.LimelightHelpers;
import swervelib.SwerveDrive;

public class LimeLocalizationSubsystem{
    public String name= "";
    private String out="";
    public LimeLocalizationSubsystem(String name){
      this.name=name;
      out= this.name.concat(" pose");
    }

    public SwerveDriveSubsystem sd;
    public void init(){
        SwerveDriveSubsystem sd=Robot.getInstance().m_robotContainer.drivebase;
    }
    public double time=0;

    public Optional<Pose2d> getPose(){
        boolean doRejectUpdate=false;
    LimelightHelpers.SetRobotOrientation("name", sd.SwerveDriveSubsystem.swerveDrivePoseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
    LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("lime1");
    if(Math.abs(sd.pigeon.getAngularVelocityYWorld().getValue()) > 720) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
    {
      doRejectUpdate = true;
    }
    if(mt2.tagCount == 0)
    {
      doRejectUpdate = true;
    }
    if(!doRejectUpdate)
    {
      //SwerveDriveSubsystem.swerveDrivePoseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
      time=mt2.timestampSeconds;
      SmartDashboard.putString(out, mt2.pose.toString());
      return Optional.of(mt2.pose);
    }
    return Optional.empty();
    }
    
}
