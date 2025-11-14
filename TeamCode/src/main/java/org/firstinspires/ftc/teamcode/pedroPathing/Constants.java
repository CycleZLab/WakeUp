package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(11.55)
            .forwardZeroPowerAcceleration(-30)
            .lateralZeroPowerAcceleration(-30)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.02, 0.0, 0.01375, 0.0))
            .headingPIDFCoefficients(new PIDFCoefficients(2.0, 0.0, 0.085, 0.0))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.008, 0.0, 0.0003, 0.6, 0.0))
            .centripetalScaling(0.0005)
            ;

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-1.85)
            .strafePodX(0)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pp")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            ;

    public static CustomSwerveConstants swerveConstants = new CustomSwerveConstants();

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .setDrivetrain(new CustomSwerveDrivetrain(hardwareMap, swerveConstants))
                .pathConstraints(pathConstraints)
                .pinpointLocalizer(localizerConstants)
                .build()
                ;
    }
}
