package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

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

    public void autoMove(double forward, double left, double power) {
        int forwardMovement = convertInchToEncoder(forward);
        int leftMovement = convertInchToEncoder(left);

        for(DcMotor motor : driveMotors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        for(DcMotor motor : strafeNeg) {
            motor.setTargetPosition(forwardMovement - leftMovement);
        }

        for(DcMotor motor : strafePos) {
            motor.setTargetPosition(forwardMovement + leftMovement);
        }

        for(DcMotor motor : driveMotors) {
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        telemetry.addData("motor target" , forwardMovement);
        telemetry.addData("leftUp current" ,leftUp.getCurrentPosition());
        telemetry.addData("rightUp current", rightUp.getCurrentPosition());
        telemetry.addData("leftDown current" ,leftDown.getCurrentPosition());
        telemetry.addData("rightDown current", rightDown.getCurrentPosition());
        telemetry.update();

        setMotorPower(power);

        waitForMotors();

        setMotorPower(0);

        for(DcMotor motor : driveMotors) {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
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

    public void turnAbsolute(double targetAngle, double speed) {
        double currentAngle = getHeading();
    }

    public void turnRelative() {
        //make this at some point soon
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

    public void orientationUpdate() {
        orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XZY, AngleUnit.DEGREES);
    }
    String properAngleFormat(AngleUnit angleUnit, double angle) {
        return properDegreeFormat(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String properDegreeFormat(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    public void controlIntakeServos(double power) {
        intakeServos[0].setPower(power);
        intakeServos[1].setPower(power);
    }



}

