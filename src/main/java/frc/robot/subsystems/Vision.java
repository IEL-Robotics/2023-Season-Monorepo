package frc.robot.subsystems;

import java.util.Collections;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//users 10 photonconfig directory

public class Vision {
    private PhotonCamera camera = new PhotonCamera("integrated_webcam");

    private static final List<AprilTag> aprilTags = Collections.unmodifiableList(List.of(
        new AprilTag(1, new Pose3d(610.77, 42.19, 18.22, new Rotation3d(0, 0, Math.toRadians(180.0)))),
        new AprilTag(2, new Pose3d(610.77, 108.19, 18.22, new Rotation3d(0, 0, Math.toRadians(180.0)))),
        new AprilTag(3, new Pose3d(610.77, 174.19, 18.22, new Rotation3d(0, 0, Math.toRadians(180.0)))),
        new AprilTag(4, new Pose3d(636.96, 265.74, 27.38, new Rotation3d(0, 0, Math.toRadians(180.0)))),
        new AprilTag(5, new Pose3d(14.25, 265.74, 27.38, new Rotation3d(0, 0, Math.toRadians(0.0)))),
        new AprilTag(6, new Pose3d(40.45, 174.19, 18.22, new Rotation3d(0, 0, Math.toRadians(0.0)))),
        new AprilTag(7, new Pose3d(40.45, 108.19, 18.22, new Rotation3d(0, 0, Math.toRadians(0.0)))),
        new AprilTag(8, new Pose3d(40.45, 42.19, 18.22, new Rotation3d(0, 0, Math.toRadians(0.0))))
    ));

    private AprilTagFieldLayout aprilTagFieldLayout = new AprilTagFieldLayout(aprilTags, 649.0, 319.0);
    private Transform3d camera2Robot = new Transform3d(new Translation3d(0.5, 0.5, 0.5),  new Rotation3d(0, 0, 0)); //madeup values

    private PhotonPipelineResult result;
    private PhotonTrackedTarget target;
    double x = 0, y = 0, angle = 0;
    double[] values = {x, y, angle};

    //Assuming AprilTag Tuning has the index 0 and Reflective Tape Tuning the index 1
    public void targetTags(){camera.setPipelineIndex(0);}

    public void targetTape(){camera.setPipelineIndex(1);}

    public double[] getFieldPosition(){
        result = camera.getLatestResult();
        if(result.hasTargets()){
            target = result.getBestTarget();
            if(target.getFiducialId() != -1){
                Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(), aprilTagFieldLayout.getTagPose(target.getFiducialId()).get(), camera2Robot);
                x = robotPose.getX();
                y = robotPose.getY();
                angle = robotPose.getRotation().getAngle();
    
                SmartDashboard.putNumber("X Pos.", x);
                SmartDashboard.putNumber("Y Pos.", y);
                SmartDashboard.putNumber("Angle", angle);// smartboard correct?
            }
        }        

        return values;
    }
    
}
