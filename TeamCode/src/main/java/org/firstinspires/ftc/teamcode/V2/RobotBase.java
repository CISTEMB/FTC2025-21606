package org.firstinspires.ftc.teamcode.V2;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.seattlesolvers.solverslib.command.CommandOpMode;

import org.firstinspires.ftc.teamcode.V2.Libs.CommandGamepad;
import org.firstinspires.ftc.teamcode.V2.Subsystems.Feeder;
import org.firstinspires.ftc.teamcode.V2.Subsystems.Hood;
import org.firstinspires.ftc.teamcode.V2.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.V2.Subsystems.Shooter;

public abstract class RobotBase extends CommandOpMode {


    //Subsystems
protected Intake intake;
protected Feeder feeder;
protected Hood hood;
protected Shooter shooter;


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




        configureButtonBindings();
    }

    protected abstract void configureButtonBindings();

    @Override
    public void run() {
        super.run();
        joinedTelemetry.update();
    }
}
