package org.firstinspires.ftc.teamcode.v2;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.v2.lib.CommandGamepad;
import org.firstinspires.ftc.teamcode.v2.lib.Commands;
import org.firstinspires.ftc.teamcode.v2.subsystems.Drive;
import org.firstinspires.ftc.teamcode.v2.subsystems.Feeder;
import org.firstinspires.ftc.teamcode.v2.subsystems.Hood;
import org.firstinspires.ftc.teamcode.v2.subsystems.Intake;
import org.firstinspires.ftc.teamcode.v2.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.v2.subsystems.Vision;

import java.util.Optional;

@Configurable
public abstract class RobotBase extends CommandOpMode {

    //
    // Constants
    //
    public static double kLLP = 0.03;

    //
    // Subsystems
    //
    protected Vision vision;
    protected Intake intake;
    protected Feeder feeder;
    protected Hood hood;
    protected Shooter shooter;
    protected Drive drive;

    //
    // OI
    //
    protected CommandGamepad commandGamepad1 = new CommandGamepad(gamepad1);
    protected CommandGamepad commandGamepad2 = new CommandGamepad(gamepad2);

    //
    // Helpers
    //
    protected JoinedTelemetry joinedTelemetry;

    @Override
    public void initialize() {
        joinedTelemetry = new JoinedTelemetry(
                PanelsTelemetry.INSTANCE.getFtcTelemetry(),
                telemetry
        );

        vision = new Vision(hardwareMap, joinedTelemetry);
        intake = new Intake(hardwareMap, joinedTelemetry);
        feeder = new Feeder(hardwareMap, joinedTelemetry);
        hood = new Hood(hardwareMap, joinedTelemetry);
        shooter = new Shooter(hardwareMap, joinedTelemetry);
        drive = new Drive(hardwareMap, joinedTelemetry);

        configureButtonBindings();
    }

    protected abstract void configureButtonBindings();

    @Override
    public void run() {
        super.run();
        joinedTelemetry.update();
    }


    //
    // Commands
    //

    public Command visionAlign() {
        return Commands.runEnd(
                ()-> {
                    // Do the P controller stuff
                    Optional<Double> angle = vision.getHorizontalAngle();
                    if (angle.isPresent()) {
                        drive.arcade(0, angle.get() * kLLP, 0);
                    } else {
                        drive.stop();
                    }
                },
                () -> drive.stop(),
                drive
        );
    }

    private double latchedRPM;
    public Command visionShoot() {
        return Commands.parallel(
            visionAlign(),
            Commands.sequence(
                vision.waitForAlignment(),
                Commands.runOnce(() -> latchedRPM = vision.getShooterRPM()),
                Commands.parallel(
                    shooter.setRPM(() -> latchedRPM),
                    Commands.sequence(
                        Commands.waitUntil(() -> shooter.isAtGoal()),
                        feeder.out()
                    )
                )
            )
        );
    }

    private Double latchedRPM2;
    public Command visionShoot2() {
        return Commands.deadline(
            Commands.sequence(
                Commands.runOnce(() -> latchedRPM2 = null),
                vision.waitForAlignment(),
                Commands.runOnce(() -> latchedRPM = vision.getShooterRPM()),
                Commands.waitUntil(() -> shooter.isAtGoal()),
                feeder.out().interruptOn(()-> shooter.hasShot())
            ),
            shooter.setRPM(() -> {
                if (latchedRPM2 == null) {
                    return 0;
                }
                return latchedRPM2;
            }),
            visionAlign()
        );
    }
}
