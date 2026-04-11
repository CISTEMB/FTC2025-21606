package org.firstinspires.ftc.teamcode.V2;

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
        Pose startpose = new Pose(63.641, 8.196, Math.toRadians(90));
        PathChain path1 = drive.getFollower().pathBuilder()
                .addPath(
                        new BezierLine(
                                startpose,
                                new Pose(59.918, 17.388)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(120))
                .build();
        drive.getFollower().setStartingPose(startpose);

    schedule(Commands.sequence(
       drive.follow(path1),
       visionShoot()
    ));
    }
}