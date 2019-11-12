package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp
public class HelpU extends CypherMethods {
    double factor = 1;
    @Override
    public void runOpMode() throws InterruptedException {
            super.runOpMode();

        waitForStart();
        while (opModeIsActive()) {
            double leftPower = gamepad1.left_stick_x * .90;
            double fowardPower = gamepad1.left_stick_y  * .90;
            double rotate = gamepad1.right_stick_x * .90;

            boolean resetServo = gamepad1.y;
            boolean servoIn = gamepad1.a;
            boolean servoOut = gamepad1.b;



            if(servoIn) {
                controlIntakeServos(1);
            } else if(servoOut) {
                controlIntakeServos(-1);
            } else if (resetServo) {
                controlIntakeServos(0);
            }

            if(gamepad1.right_bumper) {
                factor = 1;
            }
            if(gamepad1.left_bumper) {
                factor = .6;
            }

            telemetry.addData("factor", factor);
            telemetry.update();

           // if (rotate == 0) {
                manDriveMotors(fowardPower, leftPower, rotate, factor);
           // } else if(rotate != 0) {
              //  rotate(rotate * factor);
           // }

        }
    }
}






//edit