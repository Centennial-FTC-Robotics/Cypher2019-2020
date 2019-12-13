package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class mainDrive extends CypherMethods {
    private boolean inOutToggle = false;
    private boolean resetToggle = true;
    private boolean armToggle = false;
    private int prevPosition;

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        waitForStart();

        ElapsedTime speedTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
      final int checkInterval = 200;

        for(DcMotor motor : driveMotors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            prevPosition = convertEncoderToInch(motor.getCurrentPosition());
        }

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
            if (b) {
                resetToggle = !resetToggle;
            }

            if (a) {
                inOutToggle = !inOutToggle;
                resetToggle = false;
            }

            if (!resetToggle) {
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
            //Driving-------------------------------------------------------------------------------
            manDriveMotors(fowardPower, leftPower, rotate, factor);

            //Arm Control---------------------------------------------------------------------------
            if (arm) {
                armToggle = !armToggle;
            }
            if (armToggle) {
                grabServo(1);
            } else {
                grabServo(-1);
            }

            controlArm(hSlide);
            controlSlides(vSlide);

            swivelServo(swivelLeft + swivelRight);

            //Better Braking------------------------------------------------------------------------

            // if(fowardPower == 0 && leftPower == 0 && rotate == 0) {
            for (DcMotor motor : driveMotors) {
                if (speedTimer.time() > checkInterval) {
                    double speed = (double) (convertEncoderToInch(motor.getCurrentPosition()) - prevPosition) / speedTimer.time();
                    // This will print out the inches per millisecond of the motor.
                    telemetry.addData(motor.toString(), speed);
                    speedTimer.reset();
                    prevPosition = convertEncoderToInch(motor.getCurrentPosition());
                }
            }
            // }
            telemetry.update();
        }
    }
}