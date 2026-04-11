package org.firstinspires.ftc.teamcode.V2;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.util.jar.Attributes;

public abstract class BackupAuto extends AutoBase{
    @Autonomous(group = "Red", name = "RedBackUpAuto")
    public static class AutoRedBack extends BackupAuto {
        @Override
        public void initialize() {
            super.initialize();
            setRedAlliance();
        }
    }
    @Autonomous(group = "Blue", name = "BlueBackupAuto")
    public static class AutoBlueBack extends BackupAuto {
        @Override
        public void initialize(){
            super.initialize();
            setBlueAlliance();
        }
    }

    @Override
    protected void configureCommands() {
    schedule(visionShoot());
    }
}