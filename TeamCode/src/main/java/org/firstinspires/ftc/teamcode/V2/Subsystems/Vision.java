package org.firstinspires.ftc.teamcode.V2.Subsystems;

import static java.lang.Math.tan;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.V2.Libs.Commands;


public class Vision extends SubsystemBase {
    public enum Pipeline {
        kBlueOnly(1),
        kRedOnly(2),
        kBoth(3);

        public final int pipeline;
        private Pipeline(int pipeline) {
            this.pipeline = pipeline;
        }
    }


    // Hardware

    private final Limelight3A limelight;


    // State

    private final InterpLUT RPMlut = new InterpLUT();
    private LLResult lastResult;
    private final Telemetry telemetry;




    public Vision(HardwareMap hw, Telemetry telemetry) {
        limelight = hw.get(Limelight3A.class, "limelight");
        limelight.start();
        limelight.pipelineSwitch(Pipeline.kBoth.pipeline);

        // Lookup values
        RPMlut.add(-100,0);
        RPMlut.add(0, 0);
        RPMlut.add(23.6, 2500);
        RPMlut.add(29.9, 2525);
        RPMlut.add(36.2, 2550);
        RPMlut.add(52.7, 2575);
        RPMlut.add(58.6, 2580);
        RPMlut.add(65.3, 2595);
        RPMlut.add(71.6, 2625);
        RPMlut.add(77.8, 2785);
        RPMlut.add(80.0, 2835);
        RPMlut.add(81.0, 0);
        RPMlut.add(105, 0);
        RPMlut.add(110, 3505);
        RPMlut.add(125, 3515);
        RPMlut.add(135, 3545);
        RPMlut.add(1000,3555);

        RPMlut.createLUT();
        this.telemetry = telemetry;
    }

    public void setPipeline(Pipeline pipeline) {
        limelight.pipelineSwitch(pipeline.pipeline);
    }

    public boolean isValid() {
        return lastResult != null && lastResult.isValid();
    }

    public int pipeline(){
        if (!isValid()){
            return -1;
        }
        return lastResult.getPipelineIndex();
    }
    public double getHorizontalAngle() {
        if (isValid()) {
            return lastResult.getTx();
        }

        return 0;
    }

    public double getTargetDistance() {
        if (!isValid()) {
            return 0;
        }

        double h2 = 29.5;
        double h1 = 12.7127;
        double a2 = 21.9714;
        double a1 = lastResult.getTy();
        double d = (h2 - h1) / tan((a1 + a2) * 0.017453292519943295);

        return d;
    }

    public double getShooterRPM() {
        double distance = getTargetDistance();

        if (isValid()) {
            if (0<=distance && distance<=150) {
                return RPMlut.get(distance);
            }
        }

        return 0;
    }

    public boolean isAligned() {
        if(isValid()) {
            return Math.abs(getHorizontalAngle()) < 2.5;
        }

        return false;
    }

    public Command waitForAlignment() {
        return Commands.waitUntil(this::isAligned);
    }

    @Override
    public void periodic() {
        //Limelight Data
        lastResult = limelight.getLatestResult();
        telemetry.addData("Vision:Raw:isValid", lastResult.isValid());
        telemetry.addData("Vision:Raw:Pipeline", lastResult.getPipelineIndex());
        telemetry.addData("Vision:TargetDistance", getTargetDistance());
        telemetry.addData("Vision:ShooterRPM", getShooterRPM());
        telemetry.addData("Vision:Aligned", isAligned());
        telemetry.addData("Vision:Horizontal Angle", getHorizontalAngle());
        telemetry.addData("Vision:Pipeline", pipeline());
    }
}