package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Outreach")
public class OutreachDrive extends CypherMethods {

    @Override
    public void runOpMode() throws InterruptedException {
        final int miliTillReady = 250;
        super.runOpMode();
        waitForStart();

        ElapsedTime controller1Timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        ElapsedTime controller2Timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        final double factor = 0.75;
        double slideFactor = 1;
        FoundationState foundationState = FoundationState.RELEASE;
        ArmState armState = ArmState.REST;
        resetEncoders();
        resetVSlideEncoder();
        while (opModeIsActive()) {
            telemetry.addData("foundation state", foundationState);
            //controller 1 stuff
            boolean intakeIn = gamepad1.a && notInitController();
            boolean intakeOut = gamepad1.x;
            boolean intakeStop = gamepad1.b && notInitController();
            double leftPower = actualControl(gamepad1.left_stick_x, 0.7) * .2;
            double forwardPower = actualControl(gamepad1.left_stick_y, 0.6) * .4;
            double rotate = actualControl(gamepad1.right_stick_x, .6) * .2;
            //controller 2 stuff
            boolean arm = gamepad2.b && notInitController();
            boolean toggleFoundation = gamepad2.y;
            float slideSlow = gamepad2.left_trigger;
            double vSlide = gamepad2.left_stick_y;
            double hSlide = gamepad2.right_stick_x;
            boolean slowIntake;
            float slowwwwwintake = gamepad1.right_trigger;


            //timer thingy
            telemetry.addData("left slide", vLeft.getCurrentPosition());
            telemetry.addData("right slide", vRight.getCurrentPosition());
            slowIntake = slowwwwwintake > 0;
            if (controller1Timer.milliseconds() >= miliTillReady) {
                if (intakeIn) {
                    controller1Timer.reset();
                    intakeState = IntakeState.IN;
                }
                if (intakeOut) {
                    controller1Timer.reset();
                    intakeState = IntakeState.OUT;
                }
                if (intakeStop) {
                    controller1Timer.reset();
                    intakeState = IntakeState.STOP;
                }
                if (toggleFoundation) {
                    controller1Timer.reset();
                    if (foundationState.equals(FoundationState.RELEASE)) {
                        foundationState = FoundationState.DRAG;
                        controlFoundation(foundationState);
                    } else {
                        foundationState = FoundationState.RELEASE;
                        controlFoundation(foundationState);
                    }
                }
            }
            //Servo Intake Control------------------------------------------------------------------
            telemetry.addData("state", intakeState);
            switch (intakeState) {
                case IN:
                    if (!slowIntake)
                        controlIntakeMotors(0.6);
                    else
                        controlIntakeMotors(0.3);
                    changeColor(RevBlinkinLedDriver.BlinkinPattern.DARK_BLUE);
                    break;
                case OUT:
                    if (!slowIntake)
                        controlIntakeMotors(-0.6);
                    else
                        controlIntakeMotors(0.3);
                    changeColor(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
                    break;
                case STOP:
                    controlIntakeMotors(0);
                    changeColor(RevBlinkinLedDriver.BlinkinPattern.BLACK);
                    break;
            }
            //Speed Control-------------------------------------------------------------------------
            telemetry.addData("OUTREACH: Have Fun!", " ");
            telemetry.addData("factor", factor);
            //Driving-------------------------------------------------------------------------------
            manDriveMotors(forwardPower, leftPower, rotate, factor);

            //timer thingy
            if (controller2Timer.milliseconds() >= miliTillReady) {
                if (arm) {
                    controller2Timer.reset();
                    if (armState.equals(ArmState.PICK)) {
                        armState = ArmState.DROP;
                    } else {
                        armState = ArmState.PICK;
                    }
                }
            }
            switch (armState) {
                case DROP:
                    grabServo(0.6);
                    break;
                case PICK:
                    grabServo(0.2);
                    break;
            }

            if (slideSlow > 0)
                slideFactor = 0.4;
            else
                slideFactor = 1;

                //Arm Control---------------------------------------------------------------------------
                controlArm(hSlide);
                controlSlides(vSlide * slideFactor);

                telemetry.update();
            }
        }
    }
