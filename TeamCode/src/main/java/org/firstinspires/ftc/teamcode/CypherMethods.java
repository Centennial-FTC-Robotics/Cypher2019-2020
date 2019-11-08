package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import java.util.Locale;

public abstract class CypherMethods extends CypherHardware {
    private double startAngle = 0;
    private DcMotor[] driveMotors = new DcMotor[4];

    private DcMotor[] leftMotors = new DcMotor[2];
    private DcMotor[] rightMotors = new DcMotor[2];

    private DcMotor[] strafeNeg = new DcMotor[2];
    private DcMotor[] strafePos = new DcMotor[2];

    private CRServo[] intakeServos = new CRServo[2];


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
//MOVEMENT
    public void autoMove(double forward, double right, double power) {
        int forwardMovement = convertInchToEncoder(forward);
        int rightMovement = convertInchToEncoder(right);

        for (DcMotor motor : driveMotors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        leftUp.setTargetPosition(forwardMovement + rightMovement);
        rightUp.setTargetPosition(forwardMovement - rightMovement);
        leftDown.setTargetPosition(forwardMovement - rightMovement);
        rightDown.setTargetPosition(forwardMovement + rightMovement);

        for(DcMotor motor : driveMotors) {
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        setMotorPower(power);
        waitForMotors();
        setMotorPower(0);
        for(DcMotor motor : driveMotors) {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }


    public void manDriveMotors(double forwardPower, double leftPower, double rotate, double factor) {
        //double magnitude = Math.sqrt(forwardPower * forwardPower + leftPower*leftPower);
        double magnitude = Math.cbrt(forwardPower * forwardPower + leftPower*leftPower + rotate*rotate);
        if (magnitude > 1) {
           strafeNeg[0].setPower(((-leftPower + forwardPower - rotate) / magnitude) * factor);
           strafePos[0].setPower(((forwardPower + leftPower + rotate) / magnitude) * factor);
           strafePos[1].setPower(((forwardPower + leftPower - rotate) / magnitude) * factor);
           strafeNeg[1].setPower(((-leftPower + leftPower + rotate) / magnitude) * factor);
        } else {
          strafeNeg[0].setPower((-leftPower + forwardPower - rotate) * factor);
          strafePos[0].setPower((forwardPower + leftPower + rotate) * factor);
          strafePos[1].setPower((forwardPower + leftPower - rotate) * factor);
          strafeNeg[1].setPower((-leftPower + forwardPower + rotate) * factor);

        }
    }

    public void rotate(double rotate) {
        leftMotors[0].setPower(-rotate);
        leftMotors[1].setPower(-rotate);
        rightMotors[0].setPower(rotate);
        rightMotors[1].setPower(rotate);
    }

    public void turnAbsolute(double targetAngle) {
        double currentAngle = getRotationinDimension('Z');
        int direction;
        double turnRate = 0;
        double P = 0.004;
        double minSpeed = 0;
        double maxSpeed = 0.3d;
        double tolerance = 5;
        double error = getAngleDist(targetAngle, currentAngle);

        while(opModeIsActive() && error > tolerance) {
            currentAngle = getHeading();
            error = getAngleDist(targetAngle, currentAngle);
            direction = getAngleDir(targetAngle, currentAngle);
            turnRate = Range.clip(P * error, minSpeed, maxSpeed);
            telemetry.addData("error",error);
            telemetry.addData("current angle", currentAngle);
            telemetry.addData("turnRate", turnRate);
            telemetry.addData("speed", turnRate*direction);
            telemetry.update();
            setDriveMotors(-(direction * turnRate), (direction * turnRate));
        }
        setMotorPower(0);
    }

    public void setDriveMotors(double leftPower, double rightPower) {
        for (DcMotor motor : leftMotors) {
            motor.setPower(leftPower);
        }
        for (DcMotor motor : rightMotors) {
            motor.setPower(rightPower);
        }
    }


    public void setMotorPower(double power) {
        for(DcMotor motor: driveMotors) {
            motor.setPower(power);
        }
    }


    public void turnRelative() {
        //make this at some point soon
    }




    public double getHeading() {
        orientationUpdate();
        double heading = Double.parseDouble(properAngleFormat(orientation.angleUnit, orientation.firstAngle)) - startAngle;
        if(heading > 180) {
            heading -=360;
        } else if(heading < -180) {
            heading +=360;
        }
        return heading;
    }

    //INITIALIZE STUFF
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

    public void zeroAngle(){

        startAngle = getRotationinDimension('Z');
    }

    public double getRotationinDimension(char dimension) {
        switch (Character.toUpperCase(dimension)) {
            case 'X':
                    return AngleUnit.normalizeDegrees(getRawDimension('X') - initialPitch);
            case 'Y':
                return AngleUnit.normalizeDegrees(getRawDimension('Y') - initialRoll);
            case 'Z':
                return AngleUnit.normalizeDegrees(getRotationinDimension('Z') - initialHeading);
        }
        return 0;
    }

    //ANGLE-RELATED METHODS
    public double getRawDimension(char dimension) {
        orientationUpdate();
        switch(dimension) {
            case 'X':
                return orientation.secondAngle;
            case 'Y':
                return orientation.thirdAngle;
            case 'Z':
                return orientation.firstAngle;
        }
        return 0;
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


    public void orientationUpdate() {
        orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XZY, AngleUnit.DEGREES);
    }
    String properAngleFormat(AngleUnit angleUnit, double angle) {
        return properDegreeFormat(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String properDegreeFormat(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    //STATUS METHODS
    public void waitForMotors() {
        while(areMotorsBusy() && opModeIsActive()) {

        }
    }

    public boolean areMotorsBusy() {
        return leftDown.isBusy() || rightDown.isBusy() || leftUp.isBusy() || rightUp.isBusy();
    }

    //CONVERSION METHODS

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

    //INTAKE METHODS
    public void controlIntakeServos(double power) {
        intakeServos[0].setPower(power);
        intakeServos[1].setPower(power);
    }

    public void controlArm() {

    }

    public void grabServo(double power) {

    }

    public void swivelServo(double position) {

    }




}


