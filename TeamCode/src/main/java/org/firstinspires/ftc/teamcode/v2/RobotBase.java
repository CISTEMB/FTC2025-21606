package org.firstinspires.ftc.teamcode.v2;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.seattlesolvers.solverslib.command.CommandOpMode;

import org.firstinspires.ftc.teamcode.v2.lib.CommandGamepad;

public abstract class RobotBase extends CommandOpMode {

    //
    // Subsystems
    //

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

        configureButtonBindings();
    }

    protected abstract void configureButtonBindings();

    @Override
    public void run() {
        super.run();
        joinedTelemetry.update();
    }
}
