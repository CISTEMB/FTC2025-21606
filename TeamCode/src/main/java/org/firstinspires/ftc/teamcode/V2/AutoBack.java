package org.firstinspires.ftc.teamcode.V2;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.V2.Libs.Commands;


public abstract class AutoBack extends AutoBase{
    @Autonomous(group = "Red")
    public static class AutoRedBack extends AutoBack{
        @Override
        public void initialize() {
            super.initialize();
            setRedAlliance();
        }
    }
    @Autonomous(group = "Blue")
    public static class AutoBlueBack extends AutoBack{
        @Override
        public void initialize(){
            super.initialize();
            setBlueAlliance();
        }
    }

    @Override
    protected void configureCommands() {
        Follower follower = drive.getFollower();

        Pose startpose = new Pose(57.146, 8.196, Math.toRadians(90));
        PathChain drivetoShoot = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                startpose,
                                new Pose(62.482, 13.437)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(120))
                .build();

       PathChain driveAway = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(62.482, 13.437),
                                new Pose(58.981, 42.589)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

    drive.getFollower().setStartingPose(startpose);
    schedule(Commands.sequence(
        drive.follow(drivetoShoot),
        visionShoot().withTimeout(10*1000),
        drive.follow(driveAway)
    ));
    }
}