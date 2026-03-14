package org.firstinspires.ftc.teamcode.v2.subsystems;

import static java.lang.Math.tan;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.v2.lib.Commands;

import java.util.Optional;

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


    //
    // Hardware
    //
    private final Limelight3A limelight;

    //
    // State
    //
    private final InterpLUT rpmLookup = new InterpLUT();
    private LLResult lastResult;




    public Vision(HardwareMap hw, Telemetry telemetry) {
        limelight = hw.get(Limelight3A.class, "limelight");
        limelight.start();
        limelight.pipelineSwitch(Pipeline.kBoth.pipeline);

        // Lookup values
        rpmLookup.add(-100,0);
        rpmLookup.add(0, 0);
        rpmLookup.add(23.2, 2400);
        rpmLookup.add(29.5, 2550);
        rpmLookup.add(41.4, 2750);
        rpmLookup.add(52.7, 3000);
        rpmLookup.add(58.6, 3100);
        rpmLookup.add(65.3, 3215);
        rpmLookup.add(71.6, 3300);
        rpmLookup.add(77.8, 3375);
        rpmLookup.add(80.0, 3375);
        rpmLookup.add(81.0, 0);
        rpmLookup.add(105, 0);
        rpmLookup.add(110, 3515);
        rpmLookup.add(135, 3555);
        rpmLookup.add(1000,3555);

        rpmLookup.createLUT();
    }

    public Command setPipeline(Pipeline pipeline) {
        return Commands.runOnce(() -> limelight.pipelineSwitch(pipeline.pipeline));
    }

    public Optional<Double> getHorizontalAngle() {
        if (lastResult == null || !lastResult.isValid()) {
            return Optional.empty();
        }

        return Optional.of(lastResult.getTx());
    }

    public Optional<Double> getTargetDistance() {
        if (lastResult == null || !lastResult.isValid()) {
            return Optional.empty();
        }

        double h2 = 29.5;
        double h1 = 12.7127;
        double a2 = 21.9714;
        double a1 = lastResult.getTy();
        double d = (h2 - h1) / tan((a1 + a2) * 0.017453292519943295);

        return Optional.of(d);
    }

    public double getShooterRPM() {
        Optional<Double> distance = getTargetDistance();

        if (distance.isPresent()) {
            if (0<=distance.get() && distance.get()<=150) {
                return rpmLookup.get(distance.get());
            }
        }

        return 0;
    }

    public Command waitForAlignment() {
        return Commands.waitUntil( () -> {
            Optional<Double> angle = getHorizontalAngle();
            if(angle.isPresent()) {
                return Math.abs(angle.get()) < 2.5;
            }

            return false;
        });
    }

    @Override
    public void periodic() {
        //Limelight Data
        lastResult = limelight.getLatestResult();
    }
}
