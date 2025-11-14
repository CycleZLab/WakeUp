package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class CustomSwerveConstants {
    public double xVelocity = 60.2239;
    public double yVelocity = 60.1371;
    private double[] convertToPolar = Pose.cartesianToPolar(xVelocity, -yVelocity);
    public Vector frontLeftVector = new Vector(convertToPolar[0], convertToPolar[1]).normalize();

    public  double maxPower = 1;

    public  double motorCachingThreshold = 0.01;
    public  boolean useVoltageCompensation = true;
    public  double nominalVoltage = 13.0;
    public  double staticFrictionCoefficient = 0.1;

    public String leftFrontMotorName = "lfMotor";
    public String leftRearMotorName = "lrMotor";
    public String rightFrontMotorName = "rfMotor";
    public String rightRearMotorName = "rrMotor";
    public String leftFrontServoName = "lfServo";
    public String leftRearServoName = "lrServo";
    public String rightFrontServoName = "rfServo";
    public String rightRearServoName = "rrServo";
    public String leftFrontEncoderName = "lfEncoder";
    public String leftRearEncoderName = "lrEncoder";
    public String rightFrontEncoderName = "rfEncoder";
    public String rightRearEncoderName = "rrEncoder";
    public double lfEncoderMaxVoltage = 2.980;
    public double lrEncoderMaxVoltage = 3.251;
    public double rfEncoderMaxVoltage = 3.246;
    public double rrEncoderMaxVoltage = 3.252;
    public double lfEncoderMinVoltage = 0.335;
    public double lrEncoderMinVoltage = 0.006;
    public double rfEncoderMinVoltage = 0.003;
    public double rrEncoderMinVoltage = 0.006;
    public double lfEncoderOffset = 2.466;
    public double lrEncoderOffset = 3.242;
    public double rfEncoderOffset = 0.036;
    public double rrEncoderOffset = 0.616;
    public double lfServoBasePower = 0.22;
    public double lrServoBasePower = 0.22;
    public double rfServoBasePower = 0.22;
    public double rrServoBasePower = 0.22;
    public double lfRotationKP = 0.35;
    public double lrRotationKP = 0.35;
    public double rfRotationKP = 0.35;
    public double rrRotationKP = 0.35;
    public double lfRotationKI = 0.0;
    public double lrRotationKI = 0.0;
    public double rfRotationKI = 0.0;
    public double rrRotationKI = 0.0;
    public double lfRotationKD = 0.0;
    public double lrRotationKD = 0.0;
    public double rfRotationKD = 0.0;
    public double rrRotationKD = 0.0;
    public  DcMotorSimple.Direction leftFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
    public  DcMotorSimple.Direction leftRearMotorDirection = DcMotorSimple.Direction.FORWARD;
    public  DcMotorSimple.Direction rightFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
    public  DcMotorSimple.Direction rightRearMotorDirection = DcMotorSimple.Direction.FORWARD;

    public CustomSwerveConstants() {
        defaults();
    }

    public CustomSwerveConstants xVelocity(double xVelocity) {
        this.xVelocity = xVelocity;
        return this;
    }

    public CustomSwerveConstants yVelocity(double yVelocity) {
        this.yVelocity = yVelocity;
        return this;
    }

    public CustomSwerveConstants maxPower(double maxPower) {
        this.maxPower = maxPower;
        return this;
    }

    public CustomSwerveConstants leftFrontMotorName(String leftFrontMotorName) {
        this.leftFrontMotorName = leftFrontMotorName;
        return this;
    }

    public CustomSwerveConstants leftRearMotorName(String leftRearMotorName) {
        this.leftRearMotorName = leftRearMotorName;
        return this;
    }

    public CustomSwerveConstants rightFrontMotorName(String rightFrontMotorName) {
        this.rightFrontMotorName = rightFrontMotorName;
        return this;
    }

    public CustomSwerveConstants rightRearMotorName(String rightRearMotorName) {
        this.rightRearMotorName = rightRearMotorName;
        return this;
    }

    public CustomSwerveConstants leftFrontMotorDirection(DcMotorSimple.Direction leftFrontMotorDirection) {
        this.leftFrontMotorDirection = leftFrontMotorDirection;
        return this;
    }

    public CustomSwerveConstants leftRearMotorDirection(DcMotorSimple.Direction leftRearMotorDirection) {
        this.leftRearMotorDirection = leftRearMotorDirection;
        return this;
    }

    public CustomSwerveConstants rightFrontMotorDirection(DcMotorSimple.Direction rightFrontMotorDirection) {
        this.rightFrontMotorDirection = rightFrontMotorDirection;
        return this;
    }

    public CustomSwerveConstants rightRearMotorDirection(DcMotorSimple.Direction rightRearMotorDirection) {
        this.rightRearMotorDirection = rightRearMotorDirection;
        return this;
    }

    public CustomSwerveConstants motorCachingThreshold(double motorCachingThreshold) {
        this.motorCachingThreshold = motorCachingThreshold;
        return this;
    }

    public CustomSwerveConstants useVoltageCompensation(boolean useVoltageCompensation) {
        this.useVoltageCompensation = useVoltageCompensation;
        return this;
    }

    public CustomSwerveConstants nominalVoltage(double nominalVoltage) {
        this.nominalVoltage = nominalVoltage;
        return this;
    }

    public CustomSwerveConstants staticFrictionCoefficient(double staticFrictionCoefficient) {
        this.staticFrictionCoefficient = staticFrictionCoefficient;
        return this;
    }

    public double getXVelocity() {
        return xVelocity;
    }

    public void setXVelocity(double xVelocity) {
        this.xVelocity = xVelocity;
    }

    public double getYVelocity() {
        return yVelocity;
    }

    public void setYVelocity(double yVelocity) {
        this.yVelocity = yVelocity;
    }

    public Vector getFrontLeftVector() {
        return frontLeftVector;
    }

    public void setFrontLeftVector(Vector frontLeftVector) {
        this.frontLeftVector = frontLeftVector;
    }

    public double getMaxPower() {
        return maxPower;
    }

    public void setMaxPower(double maxPower) {
        this.maxPower = maxPower;
    }

    public String getLeftFrontMotorName() {
        return leftFrontMotorName;
    }

    public void setLeftFrontMotorName(String leftFrontMotorName) {
        this.leftFrontMotorName = leftFrontMotorName;
    }

    public String getLeftRearMotorName() {
        return leftRearMotorName;
    }

    public void setLeftRearMotorName(String leftRearMotorName) {
        this.leftRearMotorName = leftRearMotorName;
    }

    public String getRightFrontMotorName() {
        return rightFrontMotorName;
    }

    public void setRightFrontMotorName(String rightFrontMotorName) {
        this.rightFrontMotorName = rightFrontMotorName;
    }

    public String getRightRearMotorName() {
        return rightRearMotorName;
    }

    public void setRightRearMotorName(String rightRearMotorName) {
        this.rightRearMotorName = rightRearMotorName;
    }

    public DcMotorSimple.Direction getLeftFrontMotorDirection() {
        return leftFrontMotorDirection;
    }

    public void setLeftFrontMotorDirection(DcMotorSimple.Direction leftFrontMotorDirection) {
        this.leftFrontMotorDirection = leftFrontMotorDirection;
    }

    public DcMotorSimple.Direction getLeftRearMotorDirection() {
        return leftRearMotorDirection;
    }

    public void setLeftRearMotorDirection(DcMotorSimple.Direction leftRearMotorDirection) {
        this.leftRearMotorDirection = leftRearMotorDirection;
    }

    public DcMotorSimple.Direction getRightFrontMotorDirection() {
        return rightFrontMotorDirection;
    }

    public void setRightFrontMotorDirection(DcMotorSimple.Direction rightFrontMotorDirection) {
        this.rightFrontMotorDirection = rightFrontMotorDirection;
    }

    public DcMotorSimple.Direction getRightRearMotorDirection() {
        return rightRearMotorDirection;
    }

    public void setRightRearMotorDirection(DcMotorSimple.Direction rightRearMotorDirection) {
        this.rightRearMotorDirection = rightRearMotorDirection;
    }

    public double getMotorCachingThreshold() {
        return motorCachingThreshold;
    }

    public void setMotorCachingThreshold(double motorCachingThreshold) {
        this.motorCachingThreshold = motorCachingThreshold;
    }

    public void defaults() {
        xVelocity = 81.34056;
        yVelocity = 65.43028;
        convertToPolar = Pose.cartesianToPolar(xVelocity, -yVelocity);
        frontLeftVector = new Vector(convertToPolar[0], convertToPolar[1]).normalize();
        maxPower = 1;
        motorCachingThreshold = 0.01;
        useVoltageCompensation = true;
        nominalVoltage = 13.0;
        staticFrictionCoefficient = 0.1;
        leftFrontMotorName = "lfMotor";
        leftRearMotorName = "lrMotor";
        rightFrontMotorName = "rfMotor";
        rightRearMotorName = "rrMotor";
        leftFrontServoName = "lfServo";
        leftRearServoName = "lrServo";
        rightFrontServoName = "rfServo";
        rightRearServoName = "rrServo";
        leftFrontEncoderName = "lfEncoder";
        leftRearEncoderName = "lrEncoder";
        rightFrontEncoderName = "rfEncoder";
        rightRearEncoderName = "rrEncoder";
        lfEncoderMaxVoltage = 2.980;
        lrEncoderMaxVoltage = 3.251;
        rfEncoderMaxVoltage = 3.246;
        rrEncoderMaxVoltage = 3.252;
        lfEncoderMinVoltage = 0.335;
        lrEncoderMinVoltage = 0.006;
        rfEncoderMinVoltage = 0.003;
        rrEncoderMinVoltage = 0.006;
        lfEncoderOffset = 2.466;
        lrEncoderOffset = 3.242;
        rfEncoderOffset = 0.036;
        rrEncoderOffset = 0.616;
        lfServoBasePower = 0.22;
        lrServoBasePower = 0.22;
        rfServoBasePower = 0.22;
        rrServoBasePower = 0.22;
        lfRotationKP = 0.35;
        lrRotationKP = 0.35;
        rfRotationKP = 0.35;
        rrRotationKP = 0.35;
        lfRotationKI = 0.0;
        lrRotationKI = 0.0;
        rfRotationKI = 0.0;
        rrRotationKI = 0.0;
        lfRotationKD = 0.0;
        lrRotationKD = 0.0;
        rfRotationKD = 0.0;
        rrRotationKD = 0.0;
        leftFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
        leftRearMotorDirection = DcMotorSimple.Direction.FORWARD;
        rightFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
        rightRearMotorDirection = DcMotorSimple.Direction.FORWARD;
    }
}
