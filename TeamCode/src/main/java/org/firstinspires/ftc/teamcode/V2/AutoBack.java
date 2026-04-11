package org.firstinspires.ftc.teamcode.V2;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.V2.Libs.Commands;


public abstract class AutoBack extends AutoBase {
    @Autonomous(group = "Red")
    public static class AutoRedBack extends AutoBack {
        @Override
        public void initialize() {
            isRed = true;
            super.initialize();
            setRedAlliance();
        }
    }

    @Autonomous(group = "Blue")
    public static class AutoBlueBack extends AutoBack {
        @Override
        public void initialize() {
            isRed = false;
            super.initialize();
            setBlueAlliance();
        }
    }



    @Override
    protected void configureCommands() {
        Follower follower = drive.getFollower();
        PathChain drivetoShoot;
        PathChain driveAway;

        // Note these points are assuming the robot is on the blue side.
        Pose startpose = flipPose(57.146, 8.196, 90);
        drivetoShoot = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                startpose,
                                flipPose(57.806, 18.558),
                                flipPose(66.065, 9.630)
                        )
                )
                .setLinearHeadingInterpolation(flipAngle(90), flipAngle(120))
                .build();

        driveAway = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                flipPose(66.065, 9.630),
                                flipPose(61.844, 36.083),
                                flipPose(41.513, 36.767)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();
        drive.getFollower().

                setStartingPose(startpose);

        schedule(Commands.sequence(
                drive.follow(drivetoShoot),

                visionShoot().

                        withTimeout(10 * 1000),
                drive.follow(driveAway)
        ));
    }
}