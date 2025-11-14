package org.firstinspires.ftc.teamcode.pedroPathing;

import static com.pedropathing.math.MathFunctions.findNormalizingScaling;

import com.pedropathing.Drivetrain;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.mathUtil.PolarVector;

import java.util.Arrays;
import java.util.List;

public class CustomSwerveDrivetrain extends Drivetrain {
    private final DcMotorEx leftFrontMotor;
    private final DcMotorEx leftRearMotor;
    private final DcMotorEx rightFrontMotor;
    private final DcMotorEx rightRearMotor;
    private final CRServo leftFrontServo;
    private final CRServo leftRearServo;
    private final CRServo rightFrontServo;
    private final CRServo rightRearServo;
    private final AnalogInput leftFrontEncoder;
    private final AnalogInput leftRearEncoder;
    private final AnalogInput rightFrontEncoder;
    private final AnalogInput rightRearEncoder;
    private final List<DcMotorEx> motors;
    private final List<CRServo> servos;
    private final List<AnalogInput> encoders;
    private final VoltageSensor voltageSensor;
    private List<Double> encoderMaxVoltage;
    private List<Double> encoderMinVoltage;
    private List<Double> encoderOffset;
    private List<Double> servoBasePower;
    private List<Double> rotationKP;
    private List<Double> rotationKI;
    private List<Double> rotationKD;
    private double[] motorDirection = {1.0, 1.0, 1.0, 1.0};
    private double[] error = {0.0, 0.0, 0.0, 0.0};
    private double[] lastError = {0.0, 0.0, 0.0, 0.0};
    private double[] integral = {0.0, 0.0, 0.0, 0.0};
    private ElapsedTime[] dt = {new ElapsedTime(), new ElapsedTime(), new ElapsedTime(), new ElapsedTime()};
    private double motorCachingThreshold;
    private double staticFrictionCoefficient;
    public double[] debugArr = new double[6], truePathing = new double[4];
    public PolarVector leftSide, rightSide;

    public CustomSwerveConstants constants;

    public CustomSwerveDrivetrain(HardwareMap hardwareMap, CustomSwerveConstants swerveConstants) {
        constants = swerveConstants;

        this.motorCachingThreshold = swerveConstants.motorCachingThreshold;
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        leftFrontMotor = hardwareMap.get(DcMotorEx.class, swerveConstants.leftFrontMotorName);
        leftRearMotor = hardwareMap.get(DcMotorEx.class, swerveConstants.leftRearMotorName);
        rightFrontMotor = hardwareMap.get(DcMotorEx.class, swerveConstants.rightFrontMotorName);
        rightRearMotor = hardwareMap.get(DcMotorEx.class, swerveConstants.rightRearMotorName);
        leftFrontServo = hardwareMap.get(CRServo.class, swerveConstants.leftFrontServoName);
        leftRearServo = hardwareMap.get(CRServo.class, swerveConstants.leftRearServoName);
        rightFrontServo = hardwareMap.get(CRServo.class, swerveConstants.rightFrontServoName);
        rightRearServo = hardwareMap.get(CRServo.class, swerveConstants.rightRearServoName);
        leftFrontEncoder = hardwareMap.get(AnalogInput.class, swerveConstants.leftFrontEncoderName);
        leftRearEncoder = hardwareMap.get(AnalogInput.class, swerveConstants.leftRearEncoderName);
        rightFrontEncoder = hardwareMap.get(AnalogInput.class, swerveConstants.rightFrontEncoderName);
        rightRearEncoder = hardwareMap.get(AnalogInput.class, swerveConstants.rightRearEncoderName);

        motors = Arrays.asList(leftFrontMotor, leftRearMotor, rightFrontMotor, rightRearMotor);
        servos = Arrays.asList(leftFrontServo, leftRearServo, rightFrontServo, rightRearServo);
        encoders = Arrays.asList(leftFrontEncoder, leftRearEncoder, rightFrontEncoder, rightRearEncoder);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        setMotorsToFloat();
        breakFollowing();

        updateConstants();

        Vector copiedFrontLeftVector = swerveConstants.frontLeftVector.normalize();
        vectors = new Vector[]{
                new Vector(copiedFrontLeftVector.getMagnitude(), copiedFrontLeftVector.getTheta()),
                new Vector(copiedFrontLeftVector.getMagnitude(), 2 * Math.PI - copiedFrontLeftVector.getTheta()),
                new Vector(copiedFrontLeftVector.getMagnitude(), 2 * Math.PI - copiedFrontLeftVector.getTheta()),
                new Vector(copiedFrontLeftVector.getMagnitude(), copiedFrontLeftVector.getTheta())};
    }

    public void updateConstants() {
        leftFrontMotor.setDirection(constants.leftFrontMotorDirection);
        leftRearMotor.setDirection(constants.leftRearMotorDirection);
        rightFrontMotor.setDirection(constants.rightFrontMotorDirection);
        rightRearMotor.setDirection(constants.rightRearMotorDirection);
        encoderMaxVoltage = Arrays.asList(constants.lfEncoderMaxVoltage,
                constants.lrEncoderMaxVoltage,
                constants.rfEncoderMaxVoltage,
                constants.rrEncoderMaxVoltage);
        encoderMinVoltage = Arrays.asList(constants.lfEncoderMinVoltage,
                constants.lrEncoderMinVoltage,
                constants.rfEncoderMinVoltage,
                constants.rrEncoderMinVoltage);
        encoderOffset = Arrays.asList(constants.lfEncoderOffset,
                constants.lrEncoderOffset,
                constants.rfEncoderOffset,
                constants.rrEncoderOffset);
        servoBasePower = Arrays.asList(constants.lfServoBasePower,
                constants.lrServoBasePower,
                constants.rfServoBasePower,
                constants.rrServoBasePower);
        rotationKP = Arrays.asList(constants.lfRotationKP,
                constants.lrRotationKP,
                constants.rfRotationKP,
                constants.rrRotationKP);
        rotationKI = Arrays.asList(constants.lfRotationKI,
                constants.lrRotationKI,
                constants.rfRotationKI,
                constants.rrRotationKI);
        rotationKD = Arrays.asList(constants.lfRotationKD,
                constants.lrRotationKD,
                constants.rfRotationKD,
                constants.rrRotationKD);
        this.staticFrictionCoefficient = constants.staticFrictionCoefficient;
    }

    public void breakFollowing() {
        for (DcMotorEx motor : motors) {
            motor.setPower(0);
        }
        setMotorsToFloat();
    }

    @Override
    public void startTeleopDrive() {
        setMotorsToBrake();
    }

    @Override
    public void startTeleopDrive(boolean brakeMode) {
        setMotorsToBrake();
    }
    public double xVelocity() {
        return constants.xVelocity;
    }

    public double yVelocity() {
        return constants.yVelocity;
    }

    public void setXVelocity(double xMovement) { constants.setXVelocity(xMovement); }

    public void setYVelocity(double yMovement) { constants.setYVelocity(yMovement); }

    public double getStaticFrictionCoefficient() {
        return staticFrictionCoefficient;
    }

    @Override
    public double getVoltage() {
        return voltageSensor.getVoltage();
    }

    private double getVoltageNormalized() {
        double voltage = getVoltage();
        return (nominalVoltage - (nominalVoltage * staticFrictionCoefficient)) / (voltage - ((nominalVoltage * nominalVoltage / voltage) * staticFrictionCoefficient));
    }

    @Override
    public String debugString() {
        return "\ncorrective: " + String.format("%.2f", debugArr[0]) + ", " + String.format("%.2f", debugArr[1]) +
                "\nheading: " + String.format("%.2f", debugArr[2]) + ", " + String.format("%.2f", debugArr[3]) +
                "\npathing: " + String.format("%.2f", debugArr[4]) + ", " + String.format("%.2f", debugArr[5]) +
                "\ntp0: " + String.format("%.2f", truePathing[0]) + ", " + String.format("%.2f", truePathing[1]) +
                "\ntp1: " + String.format("%.2f", truePathing[2]) + ", " + String.format("%.2f", truePathing[3]) +
                "\nleft: " + leftSide.toString() +
                "\nright: " + rightSide.toString();
    }


    public double getWheelDirection(int index) {
        double voltage = Range.clip(encoders.get(index).getVoltage(), encoderMinVoltage.get(index), encoderMaxVoltage.get(index));
        return (((1 - ((voltage - encoderMinVoltage.get(index) - (encoderOffset.get(index) - encoderMinVoltage.get(index))) / (encoderMaxVoltage.get(index) - encoderMinVoltage.get(index)))) * 2 + 3) * Math.PI) % (2 * Math.PI) - Math.PI;
    }

    private void setMotorsToBrake() {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    private void setMotorsToFloat() {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
    }

    public double normalize (double angle) {
        double normalized = angle % (Math.PI * 2);
        if (normalized >= Math.PI) {
            normalized -= 2 * Math.PI;
        } else if (normalized < -Math.PI) {
            normalized += 2 * Math.PI;
        }
        return normalized;
    }

    public double shortest (double from, double to) {
        double difference = to - from;
        if (difference > Math.PI) {
            difference -= 2 * Math.PI;
        } else if (difference < -Math.PI) {
            difference += 2 * Math.PI;
        }
        return difference;
    }

    public void calculateError (PolarVector target, int index) {
        double error1 = shortest(getWheelDirection(index), target.getTheta());
        double error2 = shortest(getWheelDirection(index), normalize(target.getTheta()) + Math.PI);
        if (Math.abs(error1) <= Math.abs(error2)) {
            error[index] = error1;
            motorDirection[index] = 1.0;
        } else {
            error[index] = error2;
            motorDirection[index] = -1.0;
        }
    }

    public double PIDController (int index) {
        integral[index] += error[index] * dt[index].seconds();
        double servoPower = -(error[index] * rotationKP.get(index) + integral[index] * rotationKI.get(index) + (error[index] - lastError[index]) / dt[index].seconds() * rotationKD.get(index));
        dt[index].reset();
        lastError[index] = error[index];
        servoPower += servoPower > 0.1 ? servoBasePower.get(index) : servoPower < -0.1 ? -servoBasePower.get(index) : 0;
        return servoPower;
    }

    public void runModule(PolarVector target, int index) {
        calculateError(target, index);
        servos.get(index).setPower(PIDController(index));
        motors.get(index).setPower(motorDirection[index] * target.getRadius());
    }

    public double[] calculateDrive(Vector correctivePower, Vector headingPower, Vector pathingPower, double robotHeading) {
        if (correctivePower.getMagnitude() > maxPowerScaling)
            correctivePower.setMagnitude(maxPowerScaling);
        if (headingPower.getMagnitude() > maxPowerScaling)
            headingPower.setMagnitude(maxPowerScaling);
        if (pathingPower.getMagnitude() > maxPowerScaling)
            pathingPower.setMagnitude(maxPowerScaling);

        debugArr[0] = Math.toDegrees(correctivePower.getTheta());
        debugArr[1] = correctivePower.getMagnitude();
        debugArr[2] = Math.toDegrees(headingPower.getTheta());
        debugArr[3] = headingPower.getMagnitude();
        debugArr[4] = Math.toDegrees(pathingPower.getTheta());
        debugArr[5] = pathingPower.getMagnitude();

        // this contains the pathing vectors, one for each side (heading control requires 2)
        Vector[] truePathingVectors = new Vector[2];

        Vector leftSideVector = correctivePower.minus(headingPower);
        Vector rightSideVector = correctivePower.plus(headingPower);
        Vector leftSideVectorWithPathing = leftSideVector.plus(pathingPower);
        Vector rightSideVectorWithPathing = rightSideVector.plus(pathingPower);
        if (leftSideVectorWithPathing.getMagnitude() > maxPowerScaling || rightSideVectorWithPathing.getMagnitude() > maxPowerScaling) {
            truePathingVectors[0] = new Vector(maxPowerScaling, leftSideVectorWithPathing.getTheta());
            truePathingVectors[1] = new Vector(maxPowerScaling, rightSideVectorWithPathing.getTheta());
        } else {
            truePathingVectors[0] = leftSideVectorWithPathing.copy();
            truePathingVectors[1] = rightSideVectorWithPathing.copy();
        }

        truePathingVectors[0] = truePathingVectors[0].times(2.0);
        truePathingVectors[1] = truePathingVectors[1].times(2.0);

        truePathingVectors[0].rotateVector(-robotHeading);
        truePathingVectors[1].rotateVector(-robotHeading);

        truePathing[0] = Math.toDegrees(truePathingVectors[0].getTheta());
        truePathing[1] = truePathingVectors[0].getMagnitude();
        truePathing[2] = Math.toDegrees(truePathingVectors[1].getTheta());
        truePathing[3] = truePathingVectors[1].getMagnitude();

        if (voltageCompensation) {
            double voltageNormalized = getVoltageNormalized();
            for (int i = 0; i < truePathingVectors.length; i++) {
                truePathingVectors[i].setMagnitude(truePathingVectors[i].getMagnitude() * voltageNormalized);
            }
        }

        double wheelPowerMax = Math.max(Math.abs(truePathingVectors[0].getMagnitude()), Math.abs(truePathingVectors[1].getMagnitude()));

        if (wheelPowerMax > maxPowerScaling) {
            truePathingVectors[0].setMagnitude(truePathingVectors[0].getMagnitude() / wheelPowerMax * maxPowerScaling);
            truePathingVectors[1].setMagnitude(truePathingVectors[1].getMagnitude() / wheelPowerMax * maxPowerScaling);
        }

        return new double[]{truePathingVectors[0].getMagnitude(), truePathingVectors[0].getTheta(), truePathingVectors[1].getMagnitude(), truePathingVectors[1].getTheta()};
    }

    @Override
    public void runDrive(double[] drivePowers) {
        double leftTheta = 2 * Math.PI - drivePowers[1];
        double rightTheta = 2 * Math.PI - drivePowers[3];
        leftTheta = leftTheta > Math.PI ? leftTheta - 2 * Math.PI : leftTheta;
        rightTheta = rightTheta > Math.PI ? rightTheta - 2 * Math.PI : rightTheta;
        PolarVector left = new PolarVector(drivePowers[0], leftTheta);
        PolarVector right = new PolarVector(drivePowers[2], rightTheta);
        leftSide = left;
        rightSide = right;
        runModule(left, 0);
        runModule(left, 1);
        runModule(right, 2);
        runModule(right, 3);
    }

    public void setTeleOpDrive(double forward, double strafe, double turn) {
        double currentHeading = 0.0;
        double theta = Math.atan2(strafe, forward);
        double power = Math.hypot(strafe, forward);
        double realTheta = ((Math.PI * 2 + currentHeading) + theta - Math.PI) % (Math.PI * 2) - Math.PI;

        PolarVector lfResult = new PolarVector(power, realTheta).add(new PolarVector(turn, Math.PI / 4.0));
        PolarVector lrResult = new PolarVector(power, realTheta).add(new PolarVector(turn, -Math.PI / 4.0));
        PolarVector rfResult = new PolarVector(power, realTheta).add(new PolarVector(turn, Math.PI / 4.0 * 3.0));
        PolarVector rrResult = new PolarVector(power, realTheta).add(new PolarVector(turn, -Math.PI / 4.0 * 3.0));

        runModule(lfResult, 0);
        runModule(lrResult, 1);
        runModule(rfResult, 2);
        runModule(rrResult, 3);
    }
}
