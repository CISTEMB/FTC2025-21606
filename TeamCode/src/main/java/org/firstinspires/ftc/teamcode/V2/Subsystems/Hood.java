package org.firstinspires.ftc.teamcode.V2.Subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.V2.Libs.Commands;

@Configurable
public class Hood extends SubsystemBase {
    //constants
    public static  final double kDownPose = 0;
    public static  final double kUpPose = 0.35; //5 teeth

    //Hardware
    private final Servo servo;

    //Helpers
    private final Telemetry telemetry;

    public Hood(HardwareMap hw, Telemetry telemetry) {
        servo = hw.get(Servo.class, "HoodMotor");



        this.telemetry = telemetry;

    }

    public Command up() {
        return Commands.run(
                () -> servo.setPosition(kUpPose),
                this


        );
    }
    public Command down() {
        return Commands.run(
                () -> servo.setPosition(kDownPose),
                this

        );
    }

}
