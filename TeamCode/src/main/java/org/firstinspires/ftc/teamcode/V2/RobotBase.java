package org.firstinspires.ftc.teamcode.V2;

import static org.firstinspires.ftc.teamcode.V1.TeleopTest.kLLP;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.V2.Libs.CommandGamepad;
import org.firstinspires.ftc.teamcode.V2.Libs.Commands;
import org.firstinspires.ftc.teamcode.V2.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.V2.Subsystems.Feeder;
import org.firstinspires.ftc.teamcode.V2.Subsystems.Hood;
import org.firstinspires.ftc.teamcode.V2.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.V2.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.V2.Subsystems.Vision;

import java.util.Optional;

public abstract class RobotBase extends CommandOpMode {


    //Subsystems
    protected Vision vision;
    protected Intake intake;
    protected Feeder feeder;
    protected Hood hood;
    protected Shooter shooter;
    protected Drive drive;


    //OI
    protected CommandGamepad CommandGamepad1 = new CommandGamepad(gamepad1);
    protected CommandGamepad CommandGamepad2 = new CommandGamepad(gamepad2);

    //Helpers

    protected JoinedTelemetry joinedTelemetry;

    @Override
    public void initialize() {
        joinedTelemetry = new JoinedTelemetry(
        PanelsTelemetry.INSTANCE.getFtcTelemetry(),
        telemetry);

        intake = new Intake(hardwareMap, joinedTelemetry);
        feeder = new Feeder(hardwareMap, joinedTelemetry);
        hood = new Hood(hardwareMap, joinedTelemetry);
        shooter = new Shooter(hardwareMap, joinedTelemetry);
        vision = new Vision(hardwareMap, joinedTelemetry);
        drive = new Drive(hardwareMap, joinedTelemetry);






        configureButtonBindings();
    }

    protected abstract void configureButtonBindings();

    @Override
    public void run() {
        super.run();
        joinedTelemetry.update();
    }


    // Commands

    public Command visionAlign() {
        Command command;
        command = Commands.runEnd(
                () -> {
                    // Do the P controller stuff
                    Optional<Double> angle = vision.getHorizontalAngle();
                    if (angle.isPresent()) {
                        drive.arcadeDrive(0, angle.get() * kLLP, 0);
                    } else {
                        drive.stop();
                    }
                },
                () -> {
                    drive.stop();
                },
                drive
        );
        return command;
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
        Commands.waitUntil(() -> shooter.isAtGoalRPM()),
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
                    Commands.waitUntil(() -> shooter.isAtGoalRPM()),
                    feeder.out().interruptOn(()-> shooter.hasShoot())
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