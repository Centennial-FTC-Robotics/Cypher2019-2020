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
    double startAngle = 0;
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
        leftUp.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightUp.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftDown.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDown.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftUp.setPower(power);
        rightUp.setPower(power);
        leftDown.setPower(power);
        rightDown.setPower(power);
       waitForMotors();
       setMotorPower(0);
        leftUp.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightUp.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftDown.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDown.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
        double currentAngle = getRotationinDimension('Z');
        int direction;
        double turnRate = 0;
        double P = 0.004;
        double minSpeed = 0;
        double maxSpeed = 0.3d;
        double tolerance = 5;
        double error = getAngleDist(targetAngle, currentAngle);

        while(opModeIsActive() && error > tolerance) {
            currentAngle = getRotationinDimension('Z');
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
        double heading = Double.parseDouble(properAngleFormat(orientation.angleUnit, orientation.firstAngle)) - startAngle;
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


    public void zeroAngle(){

        startAngle = getRotationinDimension('Z');
    }

    public double getRotationinDimension(char dimension) {
        switch (Character.toUpperCase(dimension)) {
            case 'X':
                    return AngleUnit.normalizeDegrees(getRawX() - initialPitch);
            case 'Y':
                return AngleUnit.normalizeDegrees(getRawY() - initialRoll);
            case 'Z':
                return AngleUnit.normalizeDegrees(getRawZ() - initialHeading);
        }
        return 0;
    }

    public double getRawZ() {
        orientationUpdate();
        return orientation.firstAngle;
    }

    public double getRawY() {
        orientationUpdate();
        return orientation.thirdAngle;
    }

    public double getRawX() {
        orientationUpdate();
        return orientation.secondAngle;
    }
    public void setDriveMotors(double leftPower, double rightPower) {
        for (DcMotor motor : leftMotors) {
            motor.setPower(leftPower);
        }
        for (DcMotor motor : rightMotors) {
            motor.setPower(rightPower);
        }
        //GET MOTOR STATUS

    }
    public void waitForMotors() {
        while(areMotorsBusy() && opModeIsActive()) {

        }
    }

    public boolean areMotorsBusy() {
        return leftDown.isBusy() || rightDown.isBusy() || leftUp.isBusy() || rightUp.isBusy();
    }



}

