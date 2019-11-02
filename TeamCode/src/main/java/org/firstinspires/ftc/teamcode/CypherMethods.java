package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

public abstract class CypherMethods extends CypherHardware {
    DcMotor[] driveMotors = new DcMotor[4];

    DcMotor[] leftMotors = new DcMotor[2];
    DcMotor[] rightMotors = new DcMotor[2];

    DcMotor[] strafeNeg = new DcMotor[2];
    DcMotor[] strafePos = new DcMotor[2];

    CRServo[] intakeServos = new CRServo[2];


    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        driveMotors[0] = leftUp;
        driveMotors[1] = rightUp;
        driveMotors[2] = leftDown;
        driveMotors[3] = rightDown;

        leftMotors[0] = leftUp;
        leftMotors[1] = leftDown;
        rightMotors[0] = rightUp;
        rightMotors[1] = rightDown;

        strafeNeg[0] = leftUp;
        strafeNeg[1] = rightDown;

        strafePos[0] = rightUp;
        strafePos[1] = leftDown;

        intakeServos[0] = leftServo;
        intakeServos[1] = rightServo;

        initializeIMU();
        initialHeading = orientation.firstAngle;
        initialRoll = orientation.secondAngle;
        initialPitch = orientation.thirdAngle;


    }



    //GET MOTOR STATUS
    public void waitForMotors() {
        while(areMotorsBusy() && opModeIsActive()) {

        }
    }

    public boolean areMotorsBusy() {
        return leftDown.isBusy() || leftUp.isBusy() || rightUp.isBusy() || rightDown.isBusy();
    }
    //MOVE MOTORS IN TELEOP
    public void manDriveMotors(double forwardPower, double leftPower, double rotate, double factor) {
        //double magnitude = Math.sqrt(forwardPower * forwardPower + leftPower*leftPower);
        double magnitude = Math.cbrt(forwardPower * forwardPower + leftPower*leftPower + rotate*rotate);
        if (magnitude > 1) {
           driveMotors[0].setPower(((-leftPower + forwardPower - rotate) / magnitude) * factor);
           driveMotors[1].setPower(((forwardPower + leftPower + rotate) / magnitude) * factor);
           driveMotors[2].setPower(((forwardPower + leftPower - rotate) / magnitude) * factor);
           driveMotors[3].setPower(((-leftPower + leftPower + rotate) / magnitude) * factor);
        } else {
          driveMotors[0].setPower((-leftPower + forwardPower - rotate) * factor);
          driveMotors[1].setPower((forwardPower + leftPower + rotate) * factor);
          driveMotors[2].setPower((forwardPower + leftPower - rotate) * factor);
          driveMotors[3].setPower((-leftPower + forwardPower + rotate) * factor);

        }
    }

    //THINGS FOR AUTONOMOUS
    public void rotate(double rotate) {
        leftMotors[0].setPower(-rotate);
        leftMotors[1].setPower(-rotate);
        rightMotors[0].setPower(rotate);
        rightMotors[1].setPower(rotate);
    }


    public int convertInchToEncoder(double inches) {
        double ticksPerRotation = 383.6;
        double wheelDiameter = 3.937;
        double ticksPerWheelRotation = ticksPerRotation; //MULTIPLY BY 2 FOR ACTUAL ROBOT hktdzffd
        double distanceInWheelRotation = wheelDiameter * Math.PI;
        double ticksPerInch = distanceInWheelRotation/ticksPerWheelRotation;

        double encoderValue = inches/ticksPerInch;
        int intEncoderValue = (int) encoderValue;
        return intEncoderValue;


    }


    public void setMotorPower(double power) {
        for(DcMotor motor: driveMotors) {
            motor.setPower(power);
        }
    }



    public void turnRelative() {
        //make this at some point soon
    }



    public void controlIntakeServos(double power) {
        intakeServos[0].setPower(power);
        intakeServos[1].setPower(power);
    }

    public double getAngleDist(double targetAngle, double currentAngle) {

        double angleDifference = currentAngle - targetAngle;
        if (Math.abs(angleDifference) > 180) {
            angleDifference = 360 - Math.abs(angleDifference);
        } else {
            angleDifference = Math.abs(angleDifference);
        }

        return angleDifference;
    }

    public int getAngleDir(double targetAngle, double currentAngle) {

        double angleDifference = currentAngle - targetAngle;
        int angleDir = (int) (angleDifference / Math.abs(angleDifference));
        if (Math.abs(angleDifference) > 180) {
            angleDir *= -1;
        }
        return angleDir;
    }

    public void turnAbsolute(double targetAngle) {
        double currentAngle = getHeading();
        int direction;
        double turnrate = 0;
        double P = 1d/60d;
        double minSpeed = 0;
        double maxSpeed = 0.3d;
        double tolerance = 0.5;

        double error = getAngleDist(targetAngle, currentAngle);
        telemetry.addData("error", error);
        telemetry.update();

        while(opModeIsActive() && error > tolerance) {
            telemetry.addData("Point", 1);
            telemetry.update();
            currentAngle = getHeading();
            direction = getAngleDir(targetAngle, currentAngle);
            error = getAngleDist(targetAngle, currentAngle);
            turnrate = Range.clip(P * error, minSpeed, maxSpeed);
            telemetry.addData("turn rate", turnrate);
            telemetry.update();
            telemetry.addData("direction", direction);
            telemetry.update();
            rotate(turnrate*direction);
            telemetry.addData("point", 2);
            telemetry.update();
        }
        setMotorPower(0);
    }
    public void orientationUpdate() {
        orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XZY, AngleUnit.DEGREES);
    }
    String properAngleFormat(AngleUnit angleUnit, double angle) {
        return properDegreeFormat(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String properDegreeFormat(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
    public double getHeading() {
        orientationUpdate();
        double heading = Double.parseDouble(properAngleFormat(orientation.angleUnit, orientation.firstAngle));
        if(heading > 180) {
            heading -=360;
        } else if(heading < -180) {
            heading +=360;
        }
        return heading;
    }

    public void initializeIMU() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        telemetry.addData("are you here", true);
        telemetry.update();
        telemetry.update();
        imu.initialize(parameters);
        while (opModeIsActive() && !imu.isGyroCalibrated()) ;
        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            telemetry.addData("REEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE", "");
            telemetry.update();
        }
        orientationUpdate();
        initialHeading = orientation.firstAngle;
        initialRoll = orientation.secondAngle;
        initialPitch = orientation.thirdAngle;

    }









}

