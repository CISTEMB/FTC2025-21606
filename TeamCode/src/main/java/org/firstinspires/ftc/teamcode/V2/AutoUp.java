package org.firstinspires.ftc.teamcode.V2;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.V2.Commands.AlignWithTargetCommand;
import org.firstinspires.ftc.teamcode.V2.Libs.Commands;


public abstract class AutoUp extends AutoBase {
    @Autonomous(group = "Red")
    public static class AutoRedUp extends AutoUp {
        @Override
        public void initialize() {
            isRed = true;
            super.initialize();
            setRedAlliance();
            AlignWithTargetCommand.kShooterOffset = -2;

        }
    }

    @Autonomous(group = "Blue")
    public static class AutoBlueUp extends AutoUp {
        @Override
        public void initialize() {
            isRed = false;
            super.initialize();
            setBlueAlliance();
            AlignWithTargetCommand.kShooterOffset = 0;
        }
    }


    @Override
    protected void configureCommands() {
        Follower follower = drive.getFollower();
        PathChain drivetoShoot;
        PathChain driveAway;

        double shootOffset = 0;
        if (isRed) {
            shootOffset = -2;
        }

        // Poses Assume blue.
        Pose startpose = flipPose(29.520, 128.365, 90);
        drivetoShoot = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                startpose,
                                flipPose(48.000, 95.000)
                        )
                )
                .setLinearHeadingInterpolation(flipAngle(90), flipAngle(135))
                .build();

        driveAway = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                flipPose(31.000, 111.000),
                                flipPose(45.327, 81.366),
                                flipPose(47.638, 84.769)
                        )
                )
                .setLinearHeadingInterpolation(flipAngle(135), flipAngle(180))
                .build();
        drive.getFollower().setStartingPose(startpose);

        schedule(Commands.sequence(
                drive.follow(drivetoShoot),
                visionShoot().withTimeout(10 * 1000),
                drive.follow(driveAway)
        ));
    }
}