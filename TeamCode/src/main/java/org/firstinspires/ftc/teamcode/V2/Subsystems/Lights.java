package org.firstinspires.ftc.teamcode.V2.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.Commands;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware.GoBildaRGBIndicator;

public class Lights extends SubsystemBase {
    //Hardware
    private final GoBildaRGBIndicator left;
    private final GoBildaRGBIndicator right;

    //Helpers
    private final Telemetry telemetry;

    public Lights(HardwareMap hw, Telemetry telemetry) {
        left = new GoBildaRGBIndicator(hw, "LeftRGB");
        right = new GoBildaRGBIndicator(hw, "RightRGB");

        this.telemetry = telemetry;
    }

    public Command run(Vision vision) {
        return Commands.run(() -> {

        });
    }
}
