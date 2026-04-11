package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PredictiveBrakingCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-0.5)
            .strafePodX(4)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(12)
            .headingPIDFCoefficients(new PIDFCoefficients(1, 0, 0.01, 0.01))
            .centripetalScaling(0)
            .predictiveBrakingCoefficients(new PredictiveBrakingCoefficients(0.1, 0.09500104559045161, 0.0016480335110541853));
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("front-right")
            .rightRearMotorName("back-right")
            .leftRearMotorName("back-left")
            .leftFrontMotorName("front-left")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(66.94260580708661)
            .yVelocity(53.54782416876846);


    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();


    }
}