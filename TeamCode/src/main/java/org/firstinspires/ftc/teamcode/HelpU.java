package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp
public class HelpU extends CypherMethods {

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        
        waitForStart();
        while(opModeIsActive()) {


            double leftPower = gamepad1.left_stick_x;
            double fowardPower = gamepad1.left_stick_y;
            double rotate = gamepad1.right_stick_x;

            if (rotate != 0) {
                double magnitude = Math.max(Math.abs(leftPower + fowardPower), Math.abs(leftPower - fowardPower));
                if (magnitude > 1) {
                    moveMotors((fowardPower + leftPower) / magnitude);
                } else {
                    moveMotors(-fowardPower + leftPower);
                }
            } else {
                leftUp.setPower(rotate);
                rightDown.setPower(-rotate);
            }



            }

        }
    }





//edit