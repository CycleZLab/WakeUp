package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.SwerveDrivetrainTele;

import java.util.ArrayList;

//@Disabled
@TeleOp
public class LateralVelo extends LinearOpMode {
    SwerveDrivetrainTele drivetrain = new SwerveDrivetrainTele();

    private final ArrayList<Double> velocities = new ArrayList<>();
    public static double DISTANCE = 48;
    public static double RECORD_NUMBER = 10;
    private boolean end;

    @Override
    public void runOpMode() throws InterruptedException {
        drivetrain.init(hardwareMap);

        waitForStart();
        for (int i = 0; i < RECORD_NUMBER; i++) {
            velocities.add(0.0);
        }
        end = false;

        while (opModeIsActive()) {
//            drivetrain.drive(gamepad1);
            drivetrain.pp.update();

            if (!end) {
                if (Math.abs(drivetrain.pp.getPosY(DistanceUnit.INCH)) > DISTANCE) {
                    end = true;
                    drivetrain.setTeleOpDrive(0, 0, 0);
                } else {
                    drivetrain.setTeleOpDrive(0, -1, 0);
                    //double currentVelocity = Math.abs(follower.getVelocity().getXComponent());
                    double currentVelocity = Math.abs(drivetrain.pp.getVelY(DistanceUnit.INCH));
                    velocities.add(currentVelocity);
                    velocities.remove(0);
                }
            } else {
                drivetrain.setTeleOpDrive(0, 0, 0);
                double average = 0;
                for (double velocity : velocities) {
                    average += velocity;
                }
                average /= velocities.size();
                telemetry.addData("Lateral Velocity: ", average);

//                telemetry.update();
            }

            telemetry.addData("target heading", drivetrain.heading);
            telemetry.addData("current heading", drivetrain.currentHeading);
            telemetry.addData("dir", drivetrain.getWheelDirection());
            telemetry.addData("result", drivetrain.getWheelResults());
            telemetry.update();
        }
    }
}
