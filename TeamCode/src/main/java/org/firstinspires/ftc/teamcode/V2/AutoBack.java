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
            super.initialize();
            setRedAlliance();
        }
    }

    @Autonomous(group = "Blue")
    public static class AutoBlueBack extends AutoBack {
        @Override
        public void initialize() {
            super.initialize();
            setBlueAlliance();
        }
    }

    @Override
    protected void configureCommands() {
        Follower follower = drive.getFollower();
        PathChain drivetoShoot;
        PathChain driveAway;
        Pose startpose = new Pose(57.146, 8.196, Math.toRadians(90));
        drivetoShoot = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                startpose,
                                new Pose(57.806, 18.558),
                                new Pose(66.065, 9.630)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(120))
                .build();

        driveAway = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(66.065, 9.630),
                                new Pose(61.844, 36.083),
                                new Pose(41.513, 36.767)
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