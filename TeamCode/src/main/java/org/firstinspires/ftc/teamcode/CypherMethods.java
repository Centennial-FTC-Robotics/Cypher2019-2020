package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

public class CypherMethods extends CypherHardware {
    DcMotor[] driveMotors = new DcMotor[3];

    DcMotor[] leftMotors = new DcMotor[1];
    DcMotor[] rightMotors = new DcMotor[1];

    DcMotor[] strafeNeg = new DcMotor[1];
    DcMotor[] strafePos = new DcMotor[1];

    CRServo[] intakeServos = new CRServo[1];


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

    /*                      SIMPLE STUFF TO CHECK MOTOR STATUS                                    */
    public void waitForMotors() {
        while(areMotorsBusy() && opModeIsActive()) {

        }
    }

    public boolean areMotorsBusy() {
        return leftDown.isBusy() || leftUp.isBusy() || rightUp.isBusy() || rightDown.isBusy();
    }

    /*                         THINGS FOR TELEOP I GUESS                                          */
    public void manDriveMotors(double fowardPower, double leftPower, double factor) {
        double magnitude = Math.max(Math.abs(leftPower + fowardPower), Math.abs(leftPower - fowardPower));
        if (magnitude > 1) {
            for(DcMotor motor : strafeNeg) {
                motor.setPower(((-fowardPower + leftPower) / magnitude) * factor);
            }
            for(DcMotor motor : strafePos) {
                motor.setPower(((fowardPower + leftPower) / magnitude) * factor);
            }
        } else {
           for(DcMotor motor : strafeNeg) {
               motor.setPower((-fowardPower + leftPower) * factor);
           }
           for(DcMotor motor : strafePos) {
               motor.setPower((fowardPower + leftPower) * factor);
           }
        }
    }


    public void manRotate(double rotate, double factor) {
        leftMotors[0].setPower(rotate * factor);
        leftMotors[1].setPower(rotate * factor);
        rightMotors[0].setPower(-rotate * factor);
        rightMotors[1].setPower(-rotate * factor);
    }

    /*                     THINGS FOR AUTONONOUS I GUESS                                          */
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

    public void autoMove(double forward, double left, double power) {
        int forwardMovement = convertInchToEncoder(forward);
        int leftMovement = convertInchToEncoder(left);

        for(DcMotor motor : driveMotors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        driveMotors[0].setTargetPosition(forwardMovement - leftMovement);
        driveMotors[1].setTargetPosition(forwardMovement + leftMovement);
        driveMotors[2].setTargetPosition(forwardMovement + leftMovement);
        driveMotors[3].setTargetPosition(forwardMovement - leftMovement);

        setMotorPower(power);

        waitForMotors();

        setMotorPower(0);

        for(DcMotor motor : driveMotors) {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

    }

    public void setMotorPower(double power) {
        for(DcMotor motor: driveMotors) {
            motor.setPower(power);
        }
    }

    public void turnAbsolute() {
        //make this at some point soon
    }

    public void turnRelative() {
        //make this at some point soon
    }

    public void controlIntakeServos(double power) {
        intakeServos[0].setPower(power);
        intakeServos[1].setPower(power);
    }


}

