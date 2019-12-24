package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.lang.reflect.Array;

@TeleOp
public class mainDrive extends CypherMethods {
    private boolean armToggle = false;
    private boolean foundationToggle = false;
    private final int miliTillReady = 250  ;

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        waitForStart();
        ElapsedTime debugTimer = new ElapsedTime();
        ElapsedTime controller1Timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        ElapsedTime controller2Timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        for(DcMotor motor : driveMotors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        double factor = 1;
        IntakeState inState = IntakeState.STOP;
        FoundationState foundationState = FoundationState.RELASE;
        while (opModeIsActive()) {
            telemetry.addData("control timer", controller1Timer.milliseconds());
            double leftPower = acutalControl(gamepad1.left_stick_x) * .7;
            double fowardPower = acutalControl(gamepad1.left_stick_y) * .7;
            double rotate = acutalControl(gamepad1.right_stick_x);
            boolean toggleIntake = gamepad1.a && !(gamepad1.start || gamepad2.start);
            boolean stopIntake = gamepad1.b && !(gamepad1.start || gamepad2.start);
            double vSlide = gamepad2.left_stick_y;
            double hSlide = gamepad2.right_stick_x;
            boolean arm = gamepad2.a && !(gamepad1.start || gamepad2.start);
            double swivelRight = gamepad2.right_trigger;
            double swivelLeft = -gamepad1.left_trigger;
            boolean toggleFoundation = gamepad2.y;


            //Servo Intake Control------------------------------------------------------------------
                if (stopIntake) {
                    inState = IntakeState.STOP;
                } else if (toggleIntake && controller1Timer.milliseconds() >=  miliTillReady) {
                    controller1Timer.reset();
                    if (inState.equals(IntakeState.OUT)) {
                        inState = IntakeState.IN;
                    } else {
                        inState = IntakeState.OUT;
                    }
                }

                if(controller1Timer.milliseconds() > miliTillReady
                ) {
                    telemetry.addData("ready", true);
                } else {
                    telemetry.addData("not ready", false);
                }


            switch (inState) {
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

            if(toggleFoundation && controller2Timer.milliseconds() >= miliTillReady) {
                    if(foundationState.equals(FoundationState.RELASE)) {
                        foundationState = FoundationState.DRAG;
                        controlFoundation(foundationState);
                        telemetry.addData("in", true);
                    } else {
                        foundationState = FoundationState.RELASE;
                        controlFoundation(foundationState);
                        telemetry.addData("out", true);
                    }
            }


        }
    }
}