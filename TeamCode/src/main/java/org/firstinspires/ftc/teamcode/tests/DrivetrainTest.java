package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.SwerveDrivetrainTele;

//@Disabled
@TeleOp
public class DrivetrainTest extends LinearOpMode {
    SwerveDrivetrainTele drivetrain = new SwerveDrivetrainTele();

    @Override
    public void runOpMode() throws InterruptedException {
        drivetrain.init(hardwareMap);

        waitForStart();
        while (opModeIsActive()) {
            drivetrain.drive(gamepad1);
            telemetry.addData("target heading", drivetrain.heading);
            telemetry.addData("current heading", drivetrain.currentHeading);
            telemetry.addData("dir", drivetrain.getWheelDirection());
            telemetry.addData("result", drivetrain.getWheelResults());
            telemetry.update();
        }
    }
}
