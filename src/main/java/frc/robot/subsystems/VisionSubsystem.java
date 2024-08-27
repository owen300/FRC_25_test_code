package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.utils.LimelightHelpers;

public class VisionSubsystem extends SubsystemBase{
    AprilTagFieldLayout aprilTagFieldLayout;
    private final double visionRatio = 10;
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
        if(!limeLpose().isEmpty())sd.SwerveDriveSubsystem.addVisionMeasurement(limeLpose().get(), limeL.time,limeL.getstdev());
    }

    public void updateFromLimeR(){
        if(!limeRpose().isEmpty())sd.SwerveDriveSubsystem.addVisionMeasurement(limeRpose().get(), limeR.time,limeR.getstdev());
    }

    public void updatePhotonL(){
        Optional<EstimatedRobotPose> data=photonL.photonPose.update();
        double sum=0;
        for (PhotonTrackedTarget target :data.get().targetsUsed) {
            Translation2d tagPosition = aprilTagFieldLayout.getTagPose(target.getFiducialId()).get()
                    .getTranslation().toTranslation2d();
            sum += data.get().estimatedPose.toPose2d().getTranslation().getDistance(tagPosition);
        }
        sum/=data.get().targetsUsed.size();
    if(!data.isEmpty()){//if there is data from photon vision this runs and updates the odometry with its pose
      SmartDashboard.putString("PhotonL pose",data.get().estimatedPose.toPose2d().toString());//for testing
      sd.SwerveDriveSubsystem.addVisionMeasurement(data.get().estimatedPose.toPose2d(),data.get().timestampSeconds,getstdev(data.get().targetsUsed.size(), sum));//adds in vision
    }
    }

    public void updatePhotonR(){
        Optional<EstimatedRobotPose> data=photonR.photonPose.update();
        double sum=0;
        for (PhotonTrackedTarget target :data.get().targetsUsed) {
            Translation2d tagPosition = aprilTagFieldLayout.getTagPose(target.getFiducialId()).get()
                    .getTranslation().toTranslation2d();
            sum += data.get().estimatedPose.toPose2d().getTranslation().getDistance(tagPosition);
        }
        sum/=data.get().targetsUsed.size();
    if(!data.isEmpty()){//if there is data from photon vision this runs and updates the odometry with its pose
      SmartDashboard.putString("PhotonRmm pose",data.get().estimatedPose.toPose2d().toString());//for testing
      sd.SwerveDriveSubsystem.addVisionMeasurement(data.get().estimatedPose.toPose2d(),data.get().timestampSeconds,getstdev(data.get().targetsUsed.size(), sum));//adds in vision
    }
    }

    public  Vector<N3> getstdev(double tags, double avgDistance) {
      if (tags>1) {
        return VecBuilder.fill(0.65,0.65,0.999999);
      }else if(avgDistance>5){
          return VecBuilder.fill(0.9,0.9,0.99999);
        }else{
          if(avgDistance>4){
            return VecBuilder.fill(0.85,0.85,0.999999);
          }else if(avgDistance>3){
              return VecBuilder.fill(0.75,0.75,0.999999);
          }else if(avgDistance>2){
            return VecBuilder.fill(0.65,0.65,0.999999);
          }else{
            return VecBuilder.fill(0.6,0.6,0.999999);
          }
        }
      }


}