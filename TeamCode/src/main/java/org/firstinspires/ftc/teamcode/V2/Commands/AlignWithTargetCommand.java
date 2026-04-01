package org.firstinspires.ftc.teamcode.V2.Commands;

import com.bylazar.configurables.annotations.Configurable;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.util.Debouncer;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.V2.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.V2.Subsystems.Vision;



@Configurable
public class AlignWithTargetCommand extends CommandBase {
    public static double kLLP = 0.03;
    public static double kMinPower = 0.1;
    public static double kDebouncerSeconds = 0.25;

    private final Debouncer onTargetDebouncer = new Debouncer(kDebouncerSeconds*1000, Debouncer.DebounceType.Both);

    private final Drive drive;
    private final Vision vision;
    private final Telemetry telemetry;
    private boolean isAligned;
    //private double minPower = Double.NaN;


    public AlignWithTargetCommand(Drive drive, Vision vision, Telemetry t) {
        this.drive = drive;
        this.vision = vision;
        this.telemetry = t;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        onTargetDebouncer.reset(false);
        drive.getFollower().startTeleopDrive();
        isAligned = false;
        onTargetDebouncer.calculate(false);
//        minPower = Double.NaN;
    }

    @Override
    public void execute() {
        if (vision.isValid()) {
            double angle = vision.getHorizontalAngle();
            double turn = -angle * kLLP;
            telemetry.addData("AlignWithTarget: Turn Before", turn);
//            if (Double.isNaN(minPower)) {
//                minPower = Math.copySign(kMinPower, turn);
//            }

           //If stuck in turn
            if (Math.abs(turn) < kMinPower) {
               // turn = minPower;
                turn = Math.copySign(kMinPower, turn);
            }

            telemetry.addData("AlignWithTarget: Turn", turn);

            drive.getFollower().setTeleOpDrive(0, 0, turn, true);
        } else {
            drive.getFollower().setTeleOpDrive(0, 0, 0);
        }
        isAligned = onTargetDebouncer.calculate(vision.isAligned());
        telemetry.addData("AlignWithTarget: isAligned", isAligned);
        telemetry.addData("AlignWithTarget: P ", kLLP);
    }

    @Override
    public boolean isFinished() {
//        return onTargetDebouncer.calculate(vision.isAligned());
        return isAligned;
//        return false;
    }

    @Override
    public void end(boolean interrupted) {
        drive.getFollower().setTeleOpDrive(0, 0, 0, true);
    }
}