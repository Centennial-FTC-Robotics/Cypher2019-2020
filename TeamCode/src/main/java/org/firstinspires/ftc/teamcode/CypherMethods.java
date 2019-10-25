package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

public class CypherMethods extends CypherHardware {
    DcMotor[] motors = new DcMotor[4];
    DcMotor[] leftMotors = new DcMotor[2];
    DcMotor[] rightMotors = new DcMotor[2];

    CRServo[] intakeServos = new CRServo[2];


    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        motors[0] = leftUp;
        motors[1] = rightUp;
        motors[2] = leftDown;
        motors[3] = rightDown;

        leftMotors[0] = leftUp;
        leftMotors[1] = leftDown;
        rightMotors[0] = rightUp;
        rightMotors[1] = rightDown;

        intakeServos[0] = leftServo;
        intakeServos[1] = rightServo;


    }

    public void driveMotors(double fowardPower, double leftPower, double factor) {
        double magnitude = Math.max(Math.abs(leftPower + fowardPower), Math.abs(leftPower - fowardPower));
        if (magnitude > 1) {
            motors[0].setPower(((-fowardPower + leftPower) / magnitude) * factor);
            motors[1].setPower(((fowardPower + leftPower) / magnitude) * factor);
            motors[2].setPower(((fowardPower + leftPower) / magnitude) * factor);
            motors[3].setPower(((-fowardPower + leftPower) / magnitude) * factor);
        } else {
            motors[0].setPower((-fowardPower + leftPower) * factor);
            motors[1].setPower((fowardPower + leftPower) * factor);
            motors[2].setPower((fowardPower + leftPower) * factor);
            motors[3].setPower((-fowardPower + leftPower) * factor);
        }


    }

    public void rotateMotors(double rotate, double factor) {
        leftMotors[0].setPower(rotate * factor);
        leftMotors[1].setPower(rotate * factor);
        rightMotors[0].setPower(-rotate * factor);
        rightMotors[1].setPower(-rotate * factor);
    }


    public int convertInchToEncoder(double inches) {
        /*double ticksPerRotation = 383.6;
        double wheelDiameter = 3.937;
        double ticksPerWheelRotation = ticksPerRotation; //MULTIPLY BY 2 FOR ACTUAL ROBOT hktdzffd
        double distanceInWheelRotation = wheelDiameter * Math.PI;
        double ticksPerInch = distanceInWheelRotation/ticksPerWheelRotation;

        double encoderValue = inches/ticksPerInch;
        int intEncoderValue = (int) encoderValue;
        return intEncoderValue;
         */

        int inch = (int) inches*31;
        return inch;
    }

    public void move(double forward, double left, double power) {
        int forwardVal = convertInchToEncoder(forward);
        int leftVal = convertInchToEncoder(left);

        for(DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        leftUp.setTargetPosition(forwardVal - leftVal);
        rightUp.setTargetPosition(forwardVal + leftVal);
        leftDown.setTargetPosition(forwardVal + leftVal);
        rightDown.setTargetPosition(forwardVal - leftVal);

        for(DcMotor motor: motors) {
            motor.setPower(power);
        }
        waitForMotors();
        setMotorsPower(0);

    }
    public void waitForMotors() {
        while(areMotorsBusy() && opModeIsActive()) {

        }
    }
    public boolean areMotorsBusy() {
        return leftDown.isBusy() || leftUp.isBusy() || rightUp.isBusy() || rightDown.isBusy();
    }

    public void setMotorsPower(double power) {
        for(DcMotor motor: motors) {
            motor.setPower(0);
        }
    }

}

