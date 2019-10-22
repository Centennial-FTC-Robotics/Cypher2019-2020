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
        while (opModeIsActive()) {
            double leftPower = gamepad1.left_stick_x;
            double fowardPower = gamepad1.left_stick_y;
            double rotate = gamepad1.right_stick_x;
            double factor = 1;

            if(gamepad1.right_bumper) {
                factor = 1;
            } else if(gamepad1.left_bumper) {
                factor = .6;
            }

            if (rotate == 0) {
                driveMotors(fowardPower, leftPower, factor);
            } else if(rotate != 0) {
                rotateMotors(rotate, factor);
            }
        }
    }
}






//edit