package org.firstinspires.ftc.teamcode.V2;

public abstract class AutoBase extends RobotBase{
    @Override
    public void run() {
        super.run();
        TeleOpV2.startPose = drive.getFollower().getPose();
    }
}
