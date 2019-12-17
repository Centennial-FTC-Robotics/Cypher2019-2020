package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import java.util.Locale;

public abstract class CypherMethods extends CypherHardware
{
    DcMotor[] driveMotors = new DcMotor[4];

    private DcMotor[] leftMotors = new DcMotor[2];
    private DcMotor[] rightMotors = new DcMotor[2];

    DcMotor[] strafeNeg = new DcMotor[2];
    DcMotor[] strafePos = new DcMotor[2];

    private CRServo[] wheelIntakeServos = new CRServo[2];
    private final double ticksPerRotation = 383.6;
    private final double wheelDiameter = 3.937;
    private final double ticksPerWheelRotation = ticksPerRotation; //MULTIPLY BY 2 FOR ACTUAL ROBOT hktdzffd
    private final double distanceInWheelRotation = wheelDiameter * Math.PI;
    private final double ticksPerInch = distanceInWheelRotation/ticksPerWheelRotation;

    public enum IntakeState {
        IN, OUT, STOP
    }

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

        wheelIntakeServos[0] = leftServo;
        wheelIntakeServos[1] = rightServo;
    }

    //MOVEMENT
    /*public void autoMove(double forward, double left, double power)
    {
        int forwardMovement = convertInchToEncoder(forward);
        int leftMovement = convertInchToEncoder(left);

        for (DcMotor motor : driveMotors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        leftUp.setTargetPosition(forwardMovement - leftMovement);
        rightUp.setTargetPosition(forwardMovement + leftMovement);
        leftDown.setTargetPosition(forwardMovement + leftMovement);
        rightDown.setTargetPosition(forwardMovement - leftMovement);

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
     */

    void manDriveMotors(double forwardPower, double leftPower, double rotate, double factor)
    {
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

    void turnAbsolute(double targetAngle)
    {
        double currentAngle;
        int direction;
        double turnRate ;
        double P = 0.04;
        double minSpeed = 0.01;
        double maxSpeed = 0.4;
        double tolerance = 5;
        double error;

        do{
            currentAngle = getRotationinDimension('Z');
            error = getAngleDist(targetAngle, currentAngle);
            direction = getAngleDir(targetAngle, currentAngle);
            turnRate = Range.clip(P * error, minSpeed, maxSpeed);
            telemetry.addData("error",error);
            telemetry.addData("turnRate", turnRate);
            telemetry.addData("current", currentAngle);
            telemetry.update();
            setDriveMotors((turnRate * direction), -(turnRate * direction));
        }
        while(opModeIsActive() && error > tolerance);
        setMotorPower(0);
    }

    void testAutoMove(double forward, double left) {
        int forwardMovement = convertInchToEncoder(forward);
        int leftMovement = convertInchToEncoder(left);

        for(DcMotor motor : driveMotors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        double P = 0.04;
        double I = 0;
        double tolerance = 5;
        double minSpeed = 0.01;
        double maxSpeed = 0.5;
        double negSpeed, posSpeed;
        double currentNegPos, currentPosPos;
        double negError, posError;

        int negTarget = forwardMovement - leftMovement;
        int posTarget = forwardMovement + leftMovement;

        do {
            currentNegPos = getNegPos();
            currentPosPos = getPosPos();

            negError  =currentNegPos - negTarget;
            posError = currentPosPos - posTarget;

            negSpeed = clip(P*negError, minSpeed, maxSpeed);
            posSpeed = clip(P*posError, minSpeed, maxSpeed);

            setStrafeMotors(negSpeed, posSpeed);

            telemetry.addData("neg current", currentNegPos);
            telemetry.addData("pos current", currentPosPos);
            telemetry.addData("neg error", negError);
            telemetry.addData("pos error", posError);
            telemetry.addData("neg speed", negSpeed);
            telemetry.addData("pos speed", posSpeed);
            telemetry.addData("forward", forwardMovement);
            telemetry.addData("left", leftMovement);
            telemetry.addData("aaaaaaaaaaa","aaaaaaaaaaaaaaaaa");
            telemetry.addData("egfdg", Range.clip(-1, minSpeed, maxSpeed));
            telemetry.update();
        } while(opModeIsActive() && (Math.abs(negError) > tolerance || Math.abs(posError) > tolerance) );
        setMotorPower(0);
    }

    void selfCorrectStrafe(double forward, double left) {
        int forwardMovement = convertInchToEncoder(forward);
        int leftMovement = convertInchToEncoder(left);

        for(DcMotor motor : driveMotors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        double P = 0.04;
        double I = 0;
        double tolerance = 5;
        double angleTolerance = 10;
        double minSpeed = 0.01;
        double maxSpeed = 0.5;
        double negSpeed, posSpeed;
        double currentNegPos, currentPosPos;
        double currentAngle;
        double angleDir;
        double negError, posError;
        double angleError;
        double startAngle = getRotationinDimension('Z');

        int negTarget = forwardMovement - leftMovement;
        int posTarget = forwardMovement + leftMovement;

        do {
            currentAngle = getRotationinDimension('Z');
            angleError = currentAngle - startAngle;

            if(Math.abs(angleError) > angleTolerance) {
                turnRelative(angleError);
            }

            currentNegPos = getNegPos();
            currentPosPos = getPosPos();

            negError  =currentNegPos - negTarget;
            posError = currentPosPos - posTarget;

            negSpeed = clip(P*negError, minSpeed, maxSpeed);
            posSpeed = clip(P*posError, minSpeed, maxSpeed);

            setStrafeMotors(negSpeed, posSpeed);

            telemetry.addData("neg current", currentNegPos);
            telemetry.addData("pos current", currentPosPos);
            telemetry.addData("neg error", negError);
            telemetry.addData("pos error", posError);
            telemetry.addData("neg speed", negSpeed);
            telemetry.addData("pos speed", posSpeed);
            telemetry.addData("forward", forwardMovement);
            telemetry.addData("left", leftMovement);
            telemetry.update();
        } while(opModeIsActive() && (Math.abs(negError) > tolerance || Math.abs(posError) > tolerance) );
        setMotorPower(0);
    }


    void setDriveMotors(double leftPower, double rightPower)
    {
        for (DcMotor motor : leftMotors) {
            motor.setPower(leftPower);
        }
        for (DcMotor motor : rightMotors) {
            motor.setPower(rightPower);
        }
    }

    void setMotorPower(double power) {
        for(DcMotor motor: driveMotors) {
            motor.setPower(power);
        }
    }

    void setStrafeMotors(double neg, double pos) {
        for(DcMotor motor : strafeNeg) {
            motor.setPower(neg);
        }
        for (DcMotor motor : strafePos) {
            motor.setPower(pos);
        }
    }

    public void turnRelative(double target) {
        turnAbsolute(AngleUnit.normalizeDegrees(getRotationinDimension('Z') + target));
    }

    //INITIALIZE STUFF
    public void initializeIMU()
    {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);
        while (opModeIsActive() && !imu.isGyroCalibrated()) ;
        resetOrientation();
    }

    //METHODS THAT ASSIST WITH AUTONOMOUS IDK
    public double getRotationinDimension(char dimension) {
        orientationUpdate();
        switch (Character.toUpperCase(dimension))
        {
            case 'X':
                return AngleUnit.normalizeDegrees(getRawDimension('X') - initialPitch);
            case 'Y':
                return AngleUnit.normalizeDegrees(getRawDimension('Y') - initialRoll);
            case 'Z':
                return AngleUnit.normalizeDegrees(getRawDimension('Z') - initialHeading);
        }
        return 0;
    }

    public double getRawDimension(char dimension)
    {
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

    public double getAngleDist(double targetAngle, double currentAngle)
    {
        double angleDifference = currentAngle - targetAngle;

        if (Math.abs(angleDifference) > 180) {
            angleDifference = 360 - Math.abs(angleDifference);
        } else {
            angleDifference = Math.abs(angleDifference);
        }

        return angleDifference;
    }

    public int getAngleDir(double targetAngle, double currentAngle)
    {
        double angleDifference = targetAngle - currentAngle;
        int angleDir = (int) (angleDifference / Math.abs(angleDifference));

        if (Math.abs(angleDifference) > 180) {
            angleDir *= -1;
        }

        return angleDir;
    }

    public int getDirection(int target, int current)
    {
        int direction;
        int difference  = target - current;

        if(difference > 1) {
            direction  = 1;
        } else {
            direction = -1;
        }

        return direction;
    }

    public int getNegPos() {
        int average = 0;
        for(DcMotor motor : strafeNeg) {
            average += motor.getCurrentPosition();
        }
        return average / 2;
    }
    public int getPosPos() {
        int average = 0;
        for(DcMotor motor : strafePos) {
            average += motor.getCurrentPosition();
        }
        return average /2;
    }

    public void orientationUpdate() {
        orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }
    public void resetOrientation() {
        orientationUpdate();
        initialHeading = orientation.firstAngle;
        initialRoll = orientation.secondAngle;
        initialPitch = orientation.thirdAngle;
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

    public int averageDriveMotorEncoder()
    {
        int average = 0;
        for(DcMotor motor : driveMotors) {
            average += motor.getCurrentPosition();
        }
        return average/4;
    }

    //CONVERSION METHODS
    public int convertInchToEncoder(double inches) {
        double encoderValue = inches/ticksPerInch;
        int intEncoderValue = (int) encoderValue;
        return intEncoderValue;
    }
    public int convertEncoderToInch(int encoder) {
        double inchValue = ticksPerInch/encoder;
        int intInchValue = (int) inchValue;
        return intInchValue;
    }

    //INTAKE METHODS
     void controlIntakeServos(double power) {
        wheelIntakeServos[0].setPower(power);
        wheelIntakeServos[1].setPower(power);
    }

     void controlArm(double power) {
        HSlide.setPower(power);
    }

     void grabServo(double power) {
        arm.setPower(power);
    }

     void swivelServo(double power) {
        swivel.setPower(power);
    }

     void controlSlides(double power) {

    }

    void moveFoundation(double power) {
        foundation.setPower(power);
    }

     double acutalControl(double controller)
    {
        double a = 0.106;
        double b = controller;
        //a*b^3+(1-a)*b
        double output = (a*(Math.pow(b, 3))) + ((1-a)*b);
        return output;
    }

     double clip(double num, double min, double max)
    {
        int sign;
        if(num < 0) {
            sign = -1;
        } else {
            sign = 1;
        }
        if(Math.abs(num) < min) return min*sign;
        if(Math.abs(num) > max) return max*sign;

        return num;
    }

}


