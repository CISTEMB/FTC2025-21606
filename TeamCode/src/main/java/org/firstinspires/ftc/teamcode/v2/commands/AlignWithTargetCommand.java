package org.firstinspires.ftc.teamcode.v2.commands;

import com.bylazar.configurables.annotations.Configurable;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.util.Debouncer;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.v2.subsystems.Drive;
import org.firstinspires.ftc.teamcode.v2.subsystems.Vision;

@Configurable
public class AlignWithTargetCommand extends CommandBase {
    //
    // Constants
    //
    public static double kLLP = 0.03;
    public static double kMinPower = 0.1; // TODO TUNE

    private Debouncer alignedDebouncer = new Debouncer(0.5);

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
        drive.getFollower().startTeleopDrive();
    }

    @Override
    public void execute() {
        double turn = 0;

        // Do the P controller stuff
        if (vision.isValid()) {
            double angle = vision.getHorizontalAngle();

            // P Controller
            turn = angle * kLLP;

            if (Math.abs(turn) < kMinPower) {
                turn = Math.copySign(kMinPower, turn);
            }
        }

        drive.getFollower().setTeleOpDrive(0, 0, turn);
        t.addData("AlignWithTargetCommand: Turn", turn);
    }

    @Override
    public boolean isFinished() {
        return alignedDebouncer.calculate(vision.isAligned());
    }

    @Override
    public void end(boolean interrupted) {
        drive.getFollower().setTeleOpDrive(0,0,0);
    }
}
