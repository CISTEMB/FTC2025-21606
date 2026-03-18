package org.firstinspires.ftc.teamcode.V2.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.util.Debouncer;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.V2.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.V2.Subsystems.Vision;

import java.util.Optional;

public class AlignWIthTargetCOmmand  extends CommandBase {
public static double kLLP = 0.3;
public static kMinPower =
    private Debouncer allignedDebouncer = new Debouncer(0.5);

    private final Drive drive;
    private final Vision vision;
    private final Telemetry telemetry;

    @Override
    public void initialize() {
        super.initialize();
    }

    public AlignWIthTargetCOmmand(Drive drive, Vision vision, Drive drive1, Telemetry telemetry) {
        this.drive = drive;
        this.vision = vision;
        this.telemetry = telemetry;
    }


    @Override
    public boolean isFinished() {
        return allignedDebouncer.calculate(vision.isAligned){


    }
    @Override
    public void execute(){
            // Do the P controller stuff
        }
                    Optional<Double> angle;
        angle = vision.getHorizontalAngle();
        if (angle.isPresent()) {
                        drive.arcadeDrive(0, angle.get() * kLLP, 0);
                    } else {
                        drive.stop();
                    }
                },
                () -> {
                   drive.stop();
   },
}

