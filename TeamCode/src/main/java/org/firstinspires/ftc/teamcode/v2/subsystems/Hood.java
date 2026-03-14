package org.firstinspires.ftc.teamcode.v2.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.v2.lib.Commands;

@Configurable
public class Hood extends SubsystemBase {
    //
    // Constants
    //
    public static double kDownPosition = 0;
    public static double kUPPositon = 0.35; //5 teeth

    //
    // Hardware
    //
    private final Servo servo;

    //
    // Helpers
    //
    private final Telemetry telemetry;

    public Hood(HardwareMap hw, Telemetry telemetry) {
        servo = hw.get(Servo.class, "HoodMotor");

        this.telemetry = telemetry;
    }

    public Command up() {
        return Commands.run(
                () -> servo.setPosition(kUPPositon),
                this
        );
    }

    public Command down() {
        return Commands.run(
                () -> servo.setPosition(kDownPosition),
                this
        );
    }
}
