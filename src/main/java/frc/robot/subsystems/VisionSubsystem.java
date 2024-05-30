package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase{
    public final LimeLocalizationSubsystem limeR= new LimeLocalizationSubsystem("limeR");
    public final LimeLocalizationSubsystem limeL= new LimeLocalizationSubsystem("limeL");
    public final PhotonSubsystem photonR=new PhotonSubsystem();
    public final PhotonSubsystem photonL=new PhotonSubsystem();
    private Optional<Pose2d> limeRpose(){
        return limeR.getPose();
    }
    private Optional<Pose2d> limeLpose(){
        return limeL.getPose();
    }

    private SwerveDriveSubsystem sd;
    public void init(SwerveDriveSubsystem sd){
        this.sd=sd;
        limeL.init(sd);
        limeR.init(sd);
        photonR.init("camR");
        photonR.init("camL");
    }

    public void updateAll(){
        updateFromLimeL();
        updateFromLimeR();
        updatePhotonL();
        updatePhotonR();
    }

    public void updateFromLimeL(){
        if(!limeLpose().isEmpty())sd.SwerveDriveSubsystem.addVisionMeasurement(limeLpose().get(), limeL.time,VecBuilder.fill(.7,.7,9999999));
    }

    public void updateFromLimeR(){
        if(!limeRpose().isEmpty())sd.SwerveDriveSubsystem.addVisionMeasurement(limeRpose().get(), limeR.time,VecBuilder.fill(.7,.7,9999999));
    }

    public void updatePhotonL(){
        Optional<EstimatedRobotPose> data=photonL.photonPose.update();
    if(!data.isEmpty()){//if there is data from photon vision this runs and updates the odometry with its pose
      SmartDashboard.putString("PhotonL pose",data.get().estimatedPose.toPose2d().toString());//for testing
      sd.SwerveDriveSubsystem.addVisionMeasurement(data.get().estimatedPose.toPose2d(),data.get().timestampSeconds);//adds in vision
    }
    }

    public void updatePhotonR(){
        Optional<EstimatedRobotPose> data=photonR.photonPose.update();
    if(!data.isEmpty()){//if there is data from photon vision this runs and updates the odometry with its pose
      SmartDashboard.putString("PhotonR pose",data.get().estimatedPose.toPose2d().toString());//for testing
      sd.SwerveDriveSubsystem.addVisionMeasurement(data.get().estimatedPose.toPose2d(),data.get().timestampSeconds);//adds in vision
    }
    }
}