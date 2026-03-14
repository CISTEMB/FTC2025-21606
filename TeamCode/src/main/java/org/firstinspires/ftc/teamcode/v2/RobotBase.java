package org.firstinspires.ftc.teamcode.v2;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.seattlesolvers.solverslib.command.CommandOpMode;

import org.firstinspires.ftc.teamcode.v2.lib.CommandGamepad;
import org.firstinspires.ftc.teamcode.v2.subsystems.Feeder;
import org.firstinspires.ftc.teamcode.v2.subsystems.Intake;

public abstract class RobotBase extends CommandOpMode {

    //
    // Subsystems
    //
    protected Intake intake;
    protected Feeder feeder;

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

        intake = new Intake(hardwareMap, joinedTelemetry);
        feeder = new Feeder(hardwareMap, joinedTelemetry);

        configureButtonBindings();
    }

    protected abstract void configureButtonBindings();

    @Override
    public void run() {
        super.run();
        joinedTelemetry.update();
    }
}
