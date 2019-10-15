package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class CypherMethods extends CypherHardware {
    DcMotor[] motors = new DcMotor[4];


    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        motors[0] = leftUp;
        motors[1] = rightUp;
        motors[2] = leftDown;
        motors[3] = rightDown;

        }

    public void moveMotors(double power) {
        motors[0].setPower(-power);
        motors[1].setPower(power);
        motors[2].setPower(power);
        motors[3].setPower(-power);

    }
    }



