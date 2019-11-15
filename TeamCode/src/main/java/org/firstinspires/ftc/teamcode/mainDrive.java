package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp
public class mainDrive extends CypherMethods {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        waitForStart();
        double factor = 1;
        while (opModeIsActive()) {
            double leftPower = acutalControl(gamepad1.left_stick_x);
            double fowardPower = acutalControl(gamepad1.left_stick_y);
            double rotate = acutalControl(gamepad1.right_stick_x);

            boolean resetServo = gamepad1.y;
            boolean servoIn = gamepad1.a;
            boolean servoOut = gamepad1.b;


            if (servoIn) {
                controlIntakeServos(1);
            } else if (servoOut) {
                controlIntakeServos(-1);
            } else if (resetServo) {
                controlIntakeServos(0);
            }

            if (gamepad1.right_bumper) {
                factor = 1;
            }
            if (gamepad1.left_bumper) {
                factor = .6;
            }

            telemetry.addData("factor", factor);
            telemetry.update();

            manDriveMotors(fowardPower, leftPower, rotate, factor);


        }
    }
}
