package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import frc.robot.Constants;

public class PhotonSubsystem {
     private final AprilTagFieldLayout aprilTagFieldLayout=AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
  private PhotonCamera cam=null;
  public final PhotonPoseEstimator photonPose=new PhotonPoseEstimator(aprilTagFieldLayout,PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cam, Constants.PhotonConstants.camLocation);
    
  public void init(String n){
       cam=new PhotonCamera(n);
  }
}
