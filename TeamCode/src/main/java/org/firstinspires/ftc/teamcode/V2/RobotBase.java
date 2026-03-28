package org.firstinspires.ftc.teamcode.V2;


import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.RepeatCommand;


import org.firstinspires.ftc.teamcode.V2.Commands.AlignWithTargetCommand;
import org.firstinspires.ftc.teamcode.V2.Libs.CommandGamepad;
import org.firstinspires.ftc.teamcode.V2.Libs.Commands;
import org.firstinspires.ftc.teamcode.V2.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.V2.Subsystems.Feeder;
import org.firstinspires.ftc.teamcode.V2.Subsystems.Hood;
import org.firstinspires.ftc.teamcode.V2.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.V2.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.V2.Subsystems.Vision;
import org.firstinspires.ftc.teamcode.V2.Subsystems.Lights;

@Configurable
public abstract class RobotBase extends CommandOpMode {


    //Subsystems
    protected Vision vision;
    protected Intake intake;
    protected Feeder feeder;
    protected Hood hood;
    protected Shooter shooter;
    protected Drive drive;
    protected Lights lights;


    //OI
    protected CommandGamepad commandGamepad1;
    protected CommandGamepad commandGamepad2;

    //Helpers

    protected JoinedTelemetry joinedTelemetry;

    @Override
    public void initialize() {
        commandGamepad1 = new CommandGamepad(gamepad1);
        commandGamepad2 = new CommandGamepad(gamepad2);

        reset();
        joinedTelemetry = new JoinedTelemetry(
                PanelsTelemetry.INSTANCE.getFtcTelemetry(),
                telemetry);

        vision = new Vision(hardwareMap, joinedTelemetry);
        intake = new Intake(hardwareMap, joinedTelemetry);
        feeder = new Feeder(hardwareMap, joinedTelemetry);
        hood = new Hood(hardwareMap, joinedTelemetry);
        shooter = new Shooter(hardwareMap, joinedTelemetry);
        drive = new Drive(hardwareMap, joinedTelemetry);
        lights = new Lights(hardwareMap, joinedTelemetry);


        configureCommands();
    }

    protected abstract void configureCommands();

    @Override
    public void run() {
        super.run();
        joinedTelemetry.update();
    }


    public void setRedAlliance() {
        vision.setPipeline(Vision.Pipeline.kRedOnly);
        drive.setHeadingOffset(Math.toRadians(0));
    }

    public void setBlueAlliance() {
        vision.setPipeline(Vision.Pipeline.kBlueOnly);
        drive.setHeadingOffset(Math.toRadians(180));
    }
    // Commands

    public Command visionAlign() {
      return Commands.deadline(
              new AlignWithTargetCommand(drive, vision, joinedTelemetry),
              shooter.setRPM(2400)
        ); 
    }


    private double latchedRPM;
    private double latchedDistance;


    public Command visionShoot() {
        return Commands.sequence(
                visionAlign(),
                Commands.runOnce(() -> {
                    latchedDistance = vision.getTargetDistance();
                    latchedRPM = vision.getShooterRPM();
                }),
                new RepeatCommand(
                        Commands.race(
                                shooter.setRPM(() -> latchedRPM),
                                new ConditionalCommand(
                                        hood.up(),
                                        hood.down(),
                                        () -> latchedDistance > 110
                                ),
                                Commands.sequence(
                                        Commands.waitUntil(() -> shooter.isAtGoalRPM()),
                                        Commands.parallel(
                                            intake.feed(),
                                            feeder.in()
                                        ).interruptOn(()-> shooter.hasShoot())
                                )
                        )
                )
        );

    }
}