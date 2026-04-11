package org.firstinspires.ftc.teamcode.V2;

import com.pedropathing.geometry.Pose;

public abstract class AutoBase extends RobotBase{
    @Override
    public void run() {
        super.run();
        TeleOpV2.startPose = drive.getFollower().getPose();
    }

    public Pose flipPose(Pose pose)  {
        if (isRed) {
            return new Pose(
                    144.0 - pose.getX(),
                    pose.getY(),
                    flipAngle(Math.toDegrees(pose.getHeading()))
            );
        } else {
            return pose;
        }
    }

    public Pose flipPose(double x, double y) {
        return flipPose(new Pose(x, y));
    }

    public Pose flipPose(double x, double y, double degrees) {
        return flipPose(new Pose(x, y, Math.toRadians(degrees)));
    }

    double flipAngle(double degrees) {
     if (isRed){
         return Math.toRadians(180-degrees);
     } else {
         return Math.toRadians(degrees);
     }
    }
}
