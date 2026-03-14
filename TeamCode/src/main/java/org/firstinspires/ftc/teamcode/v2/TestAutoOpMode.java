package org.firstinspires.ftc.teamcode.v2;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.v2.lib.Commands;
import org.firstinspires.ftc.teamcode.v2.subsystems.Vision;

@TeleOp(name="TeleOp V2")
public class TestAutoOpMode extends RobotBase {

    @Override
    protected void configureButtonBindings() {
        schedule(Commands.sequence(
                // DRIVE TO POSTION via pedo pathing
                visionShoot2(),
                visionShoot2(),
                visionShoot2()
                // DRIVE TO somehwere else
        ));
    }
}
