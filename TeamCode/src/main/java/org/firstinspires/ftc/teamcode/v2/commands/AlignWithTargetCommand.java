package org.firstinspires.ftc.teamcode.v2.commands;

import com.bylazar.configurables.annotations.Configurable;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.util.Debouncer;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.v2.subsystems.Drive;
import org.firstinspires.ftc.teamcode.v2.subsystems.Vision;

// Really old example(s):
// - https://github.com/CISTEMB/FTC2022-7000-RObot/blob/main/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/commands/TurnInPlace.java
// - https://github.com/KennedyRoboEagles/FTC2021-FreightFrenzy/blob/develop/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/commands/TurnInPlace.java
// - https://github.com/KennedyRoboEagles/FTC2020-Skystone/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/commands/TurnSpecifiedDegressCommand.java
// - https://github.com/KennedyRoboEagles/PublicRobotCode/blob/master/2019/Robot2019/src/main/java/frc/robot/commands/motion/TurnToAngleCommand.java
// - https://github.com/KennedyRoboEagles/PublicRobotCode/blob/master/2019/Robot2019/src/main/java/frc/robot/commands/motion/RotateToAngleCommand.java
// - https://github.com/KennedyRoboEagles/PublicRobotCode/blob/master/2018/2018Robot/src/Commands/Motion/RotateToAngle.cpp
// - https://github.com/KennedyRoboEagles/PublicRobotCode/blob/master/2017/2017Robot/src/Commands/MotionProfiling/TurnSpecifiedDegreesCommand.cpp
// - https://github.com/KennedyRoboEagles/PublicRobotCode/blob/master/2017/2017Robot/src/Commands/MotionProfiling/TurnSpecifiedDegrees2Command.cpp
// - https://github.com/KennedyRoboEagles/PublicRobotCode/blob/master/2016/Robot2016/src/com/kennedyrobotics/commands/drive/TurnSpecifiedDegreesCommand.java
// - https://github.com/KennedyRoboEagles/PublicRobotCode/blob/master/2015/2015Robot/src/Commands/TurnSpecifiedDegreesCommand.cpp
// - https://github.com/KennedyRoboEagles/PublicRobotCode/blob/master/2014/2014Robot/Commands/TurnSpecifiedDegreesCommand.cpp
// - https://github.com/KennedyRoboEagles/PublicRobotCode/blob/master/2014/2014Robot/Commands/TurnSpecifiedDegreesEncoderCountCommand.cpp
// - https://github.com/KennedyRoboEagles/PublicRobotCode/blob/master/2014/2014Robot/Commands/TurnSpecifiedDegreesPIDCommand.cpp
// - https://github.com/KennedyRoboEagles/PublicRobotCode/blob/master/2013/2013Robot/Commands/TurnSpecifiedDegreesCommand.cpp
// - https://github.com/KennedyRoboEagles/PublicRobotCode/blob/master/2013/2013Robot/Commands/TurnSpecifiedDegreesPIDCommand.cpp
// - https://github.com/KennedyRoboEagles/PublicRobotCode/blob/master/2012/2012RobotProgram/Commands/TurnSpecifiedDegreesCommand.cpp


@Configurable
public class AlignWithTargetCommand extends CommandBase {
    public static double kLLP = 0.03;
    public static double kMinPower = 0.2; // TODO Tune

    private final Debouncer onTargetDebouncer = new Debouncer(0.25);

    private final Drive drive;
    private final Vision vision;
    private final Telemetry t;


    public AlignWithTargetCommand(Drive drive, Vision vision, Telemetry t) {
        this.drive = drive;
        this.vision = vision;
        this.t = t;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        onTargetDebouncer.reset(false);
        drive.getFollower().startTeleopDrive();
    }

    @Override
    public void execute() {
        if (vision.isValid()) {
            double angle = vision.getHorizontalAngle();
            double turn = angle * kLLP;

            // Some tricks to make is to the robot doesn't get stuck if it needs to still turn
            if (Math.abs(turn) < kMinPower) {
                turn = Math.copySign(kMinPower, turn);
            }

            t.addData("ALignWithTarget: Turn", turn);

            drive.getFollower().setTeleOpDrive(0, 0, turn, true);
        }

    }

    @Override
    public boolean isFinished() {
        // The debouncer will make sure we are aligned for 0.25 seconds and will reset if
        // it flickers
        // Example usage (last two lines are unrelated to the debouncer, but getting this robot to face right away):
        // - https://github.com/Team2470/FRC-2024-Robot/blob/2c670fd65e807938668e4fef5d306883b1f57c47/src/main/java/frc/robot/RobotContainer.java#L673
        // - https://github.com/Team2470/FRC-2024-Robot/blob/2c670fd65e807938668e4fef5d306883b1f57c47/src/main/java/frc/robot/RobotContainer.java#L669-L678
        // - https://github.com/Team2470/FRC-2024-Robot/blob/2c670fd65e807938668e4fef5d306883b1f57c47/src/main/java/frc/robot/RobotContainer.java#L405-L413
        // - https://github.com/Team2470/FRC-2024-Robot/blob/2c670fd65e807938668e4fef5d306883b1f57c47/src/main/java/frc/robot/commands/DriveWithController.java#L176-L190
        return onTargetDebouncer.calculate(vision.isAligned());
    }

    @Override
    public void end(boolean interrupted) {
        drive.getFollower().setTeleOpDrive(0, 0, 0, true);
    }
}
