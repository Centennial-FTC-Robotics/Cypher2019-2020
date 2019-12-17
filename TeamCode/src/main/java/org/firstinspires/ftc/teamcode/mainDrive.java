package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.lang.reflect.Array;

@TeleOp
public class mainDrive extends CypherMethods {
    private boolean inOutToggle = false;
    private boolean resetToggle = true;
    private boolean armToggle = false;
    private int[] prevPosition = new int[4];




    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        waitForStart();
        ElapsedTime controller1Timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        ElapsedTime controller2Timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        final int checkInterval = 1;
        int i = 0;
        for(DcMotor motor : driveMotors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            prevPosition[i] = convertEncoderToInch(motor.getCurrentPosition());
            i++;
        }

        double factor = 1;
        IntakeState state = IntakeState.STOP;
        while (opModeIsActive()) {
            double leftPower = acutalControl(gamepad1.left_stick_x);
            double fowardPower = acutalControl(gamepad1.left_stick_y);
            double rotate = acutalControl(gamepad1.right_stick_x);
            boolean a = gamepad1.a && (!gamepad1.start || !gamepad2.start);
            boolean b = gamepad1.b && (!gamepad1.start || !gamepad2.start);
            double vSlide = gamepad2.left_stick_y;
            double hSlide = gamepad2.right_stick_x;
            boolean arm = gamepad2.a && (!gamepad1.start || !gamepad2.start);
            double swivelRight = gamepad2.right_trigger;
            double swivelLeft = -gamepad1.left_trigger;

            //Servo Intake Control------------------------------------------------------------------
            if(controller1Timer.time() > 450) {
                controller1Timer.reset();
                if (b) {
                    state = IntakeState.STOP;
                } else if (a) {
                    if (state.equals(IntakeState.IN)) {
                        state = IntakeState.OUT;
                    } else {
                        state = IntakeState.IN;
                    }
                }
            }

            switch (state) {
                case IN:
                    controlIntakeServos(1);
                    break;
                case OUT:
                    controlIntakeServos(-1);
                    break;
                case STOP:
                    controlIntakeServos(0);
                    break;
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
            if (arm && controller2Timer.time() < 450) {
                controller2Timer.reset();
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

            telemetry.update();
        }
    }
}