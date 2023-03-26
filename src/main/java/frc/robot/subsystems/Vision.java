package frc.robot.subsystems;

import java.io.IOException;
// import java.util.Collections;
// import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//users 10 photonconfig directory

public class Vision {
    private PhotonCamera camera = new PhotonCamera("ielrobotik_number_1");

    private AprilTagFieldLayout aprilTagFieldLayout;

    private Transform3d camera2Robot = new Transform3d(new Translation3d(0.5, 0.5, 0.5), new Rotation3d(0, 0, 0)); // madeup                                                                                                          // values

    private PhotonPipelineResult result;
    private PhotonTrackedTarget target;
    double x = 0, y = 0, angle = 0;
    double[] values = { x, y, angle };

    public Vision() {
        try {
            this.aprilTagFieldLayout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    // Assuming AprilTag Tuning has the index 0 and Reflective Tape Tuning the index
    // 1
    public void targetTags() {
        camera.setPipelineIndex(0);
    }

    public void targetTape() {
        camera.setPipelineIndex(1);
    }

    public double[] getFieldPosition() {
        result = camera.getLatestResult();
        if (result.hasTargets()) {
            target = result.getBestTarget();
            if (target.getFiducialId() != -1) {
                Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(),
                        aprilTagFieldLayout.getTagPose(target.getFiducialId()).get(), camera2Robot);
                x = robotPose.getX();
                y = robotPose.getY();
                angle = robotPose.getRotation().getAngle();

                SmartDashboard.putNumber("AprilTag ID:", target.getFiducialId());
                SmartDashboard.putNumber("VISION X Pos.", x);
                SmartDashboard.putNumber("VISION Y Pos.", y);
                SmartDashboard.putNumber("VISION Angle", angle);// smartboard correct?
            }
        }

        SmartDashboard.putNumber("VISION X Pos.", x);
        SmartDashboard.putNumber("VISION Y Pos.", y);
        SmartDashboard.putNumber("VISION Angle", angle);

        return values;
    }

}
