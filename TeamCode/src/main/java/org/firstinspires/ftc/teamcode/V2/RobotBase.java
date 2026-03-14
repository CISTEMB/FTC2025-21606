package org.firstinspires.ftc.teamcode.V2;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.seattlesolvers.solverslib.command.CommandOpMode;

import org.firstinspires.ftc.teamcode.V2.Libs.CommandGamepad;

public abstract class RobotBase extends CommandOpMode {


    //Subsystems


    //OI
    protected CommandGamepad CommmandGamepad1 = new CommandGamepad(gamepad1);
    protected CommandGamepad CommmandGamepad2 = new CommandGamepad(gamepad2);

    //Helpers
    protected JoinedTelemetry joinedTelemetry;

    @Override
    public void initialize() {
        joinedTelemetry = new JoinedTelemetry(
        PanelsTelemetry.INSTANCE.getFtcTelemetry(),
        telemetry);


        configureButtonBindings();
    }

    protected abstract void configureButtonBindings();

    @Override
    public void run() {
        super.run();

        joinedTelemetry.update();
    }
}
