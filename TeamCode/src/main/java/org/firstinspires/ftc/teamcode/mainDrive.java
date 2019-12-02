package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;


@TeleOp
public class mainDrive extends CypherMethods {
    private boolean inOutToggle = false;
    private boolean resetToggle = true;
    private boolean armToggle = false;
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        waitForStart();
        double factor = 1;
        while (opModeIsActive()) {
            double leftPower = acutalControl(gamepad1.left_stick_x);
            double fowardPower = acutalControl(gamepad1.left_stick_y);
            double rotate = acutalControl(gamepad1.right_stick_x);
            boolean a = gamepad1.a;
            boolean b = gamepad1.b;
            double vSlide = gamepad2.left_stick_y;
            double hSlide = gamepad2.right_stick_x;
            boolean arm = gamepad2.a;
            double swivelRight = gamepad2.right_trigger;
            double swivelLeft = -gamepad1.left_trigger;
            //Servo Intake Control------------------------------------------------------------------
            if(b) {
                resetToggle = !resetToggle;
            }

            if (a) {
                inOutToggle = !inOutToggle;
                resetToggle = false;
            }

            if(!resetToggle) {
                if (inOutToggle) {
                    controlIntakeServos(1);
                } else if (!inOutToggle) {
                    controlIntakeServos(-1);
                } else {
                    controlIntakeServos(0);
                }
            } else {
                controlIntakeServos(0);
            }


            //Speed Control-------------------------------------------------------------------------
            if (gamepad1.right_bumper) {
                factor = 1;
            }
            if (gamepad1.left_bumper) {
                factor = .6;
            }

            telemetry.addData("factor", factor);
            telemetry.update();

            //Driving-------------------------------------------------------------------------------
            manDriveMotors(fowardPower, leftPower, rotate, factor);

            //Arm Control---------------------------------------------------------------------------
            if(arm) {
                armToggle = !armToggle;
            }
            if(armToggle) {
                grabServo(1);
            } else {
                grabServo(-1);
            }

            controlArm(hSlide);
            controlSlides(vSlide);

            swivelServo(Range.clip(swivelLeft + swivelRight, -1, 1));




        }
    }
}
