package frc.robot.subsystems;

import java.util.Optional;

// import org.photonvision.EstimatedRobotPose;
// import org.photonvision.targeting.PhotonTrackedTarget;

//import edu.wpi.first.apriltag.AprilTagFieldLayout;
// import edu.wpi.first.math.VecBuilder;
// import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.numbers.N3;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase{//belive it or not, this class has no warnings
  //despite the crimes against code committed here
    //AprilTagFieldLayout aprilTagFieldLayout;//used for finding distance from 2 tags
    public final LimeLocalizationSubsystem limeF= new LimeLocalizationSubsystem("limeF");
    public final LimeLocalizationSubsystem limeB= new LimeLocalizationSubsystem("limeB");//lime names passed here
    //public final PhotonSubsystem photonR=new PhotonSubsystem();
    //public final PhotonSubsystem photonL=new PhotonSubsystem();

    private Optional<Pose2d> limeFpose(){
        return limeF.getPose();//getter method for limelight subsystem
    }
    private Optional<Pose2d> limeBpose(){
        return limeB.getPose();//getter method for limelight subsystem
    }

    private SwerveDriveSubsystem sd;
    public void init(SwerveDriveSubsystem sd){
        this.sd=sd;
        limeF.init(sd);
        limeB.init(sd);//gives swerve subsystem bc it needs imu measurements
        //photonR.init("camR");
        //photonR.init("camL");//camera names for photon
    }

    public void updateAll(){//update all cameras
        updateFromLimeF();
        updateFromLimeB();
        //updatePhotonL();
        //updatePhotonR();
    }

    public void updateFromLimeF(){
        if(!limeFpose().isEmpty())sd.SwerveDriveSubsystem.addVisionMeasurement(limeFpose().get(), limeF.time,limeF.getstdev());//add limelight in
    }

    public void updateFromLimeB(){
        if(!limeBpose().isEmpty())sd.SwerveDriveSubsystem.addVisionMeasurement(limeBpose().get(), limeB.time,limeB.getstdev());//add limelight
    }

//commenting out photon as i do not think 4 cameras will be an advantage
//also comments out some of the worst code if done that badly needs organizing to keep using
    // public void updatePhotonL(){
    //     Optional<EstimatedRobotPose> data=photonL.photonPose.update();
    // if(!data.isEmpty()){//if there is data from photon vision this runs and updates the odometry with its pose
    //   double sum=0;
    //   for (PhotonTrackedTarget target :data.get().targetsUsed) {//complicated thing to find avg tag distance
    //       Translation2d tagPosition = aprilTagFieldLayout.getTagPose(target.getFiducialId()).get()
    //               .getTranslation().toTranslation2d();
    //       sum += data.get().estimatedPose.toPose2d().getTranslation().getDistance(tagPosition);
    //   }
    //   sum/=data.get().targetsUsed.size();
    //   SmartDashboard.putString("PhotonL pose",data.get().estimatedPose.toPose2d().toString());//for testing
    //   sd.SwerveDriveSubsystem.addVisionMeasurement(data.get().estimatedPose.toPose2d(),data.get().timestampSeconds,getstdev(data.get().targetsUsed.size(), sum));//adds in vision
    // }
    // }

    // public void updatePhotonR(){
    //     Optional<EstimatedRobotPose> data=photonR.photonPose.update();
    // if(!data.isEmpty()){//if there is data from photon vision this runs and updates the odometry with its pose
    //   double sum=0;
    //     for (PhotonTrackedTarget target :data.get().targetsUsed) {//complicated thing to find avg tag distance
    //         Translation2d tagPosition = aprilTagFieldLayout.getTagPose(target.getFiducialId()).get()
    //                 .getTranslation().toTranslation2d();
    //         sum += data.get().estimatedPose.toPose2d().getTranslation().getDistance(tagPosition);
    //     }
    //     sum/=data.get().targetsUsed.size();
    //   SmartDashboard.putString("PhotonRmm pose",data.get().estimatedPose.toPose2d().toString());//for testing
    //   sd.SwerveDriveSubsystem.addVisionMeasurement(data.get().estimatedPose.toPose2d(),data.get().timestampSeconds,getstdev(data.get().targetsUsed.size(), sum));//adds in vision
    // }
    // }

    // public  Vector<N3> getstdev(double tags, double avgDistance) {//kinda janky system to hopefully tune out noisy measurements
    //   if (tags>1) {//if there is more than one tag we just trust the localization beacause
    //     return VecBuilder.fill(0.65,0.65,0.999999);//i think it should be good enough
    //   }else if(avgDistance>5){
    //       return VecBuilder.fill(0.9,0.9,0.99999);//if its over 5 meters out we dont trust vision very much
    //     }else{
    //       if(avgDistance>4){
    //         return VecBuilder.fill(0.85,0.85,0.999999);//over 4 we trust a little
    //       }else if(avgDistance>3){
    //           return VecBuilder.fill(0.725,0.725,0.999999);//over 3 a decent amount of trust
    //       }else if(avgDistance>2){
    //         return VecBuilder.fill(0.65,0.65,0.999999);//almost full trust if over 2
    //       }else{
    //         return VecBuilder.fill(0.6,0.6,0.999999);//below 2 is the maximum trust im putting in our cameras
    //       }//this whole thing might be lowkey useless
    //     }
    //   }


}