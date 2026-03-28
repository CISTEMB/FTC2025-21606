package org.firstinspires.ftc.teamcode.V2;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

public abstract class AutoBack extends AutoBase{
    @Autonomous(group = "Red")
    public static class AutoRedBack extends AutoBack{
        @Override
        public void initialize() {
            super.initialize();
            setRedAlliance();
        }
    }
    @Autonomous(group = "Blue")
    public static class AutoBlueBack extends AutoBack{
        @Override
        public void initialize(){
            super.initialize();
            setBlueAlliance();
        }
    }

    @Override
    protected void configureCommands() {

    }
}