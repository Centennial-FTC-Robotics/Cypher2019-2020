package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class mainDrive extends CypherMethods {

    @Override
    public void runOpMode() throws InterruptedException {
        final int miliTillReady = 250;
        super.runOpMode();
        waitForStart();
        ElapsedTime controller1Timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        ElapsedTime controller2Timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        double factor;
        double slideFactor = 1;
        IntakeState inState = IntakeState.STOP;
        FoundationState foundationState = FoundationState.RELEASE;
        ArmState armState = ArmState.REST;
        boolean intakeIn, intakeOut, intakeStop;
        double leftPower, forwardPower, rotate, driveSlow, slideSlow;
        boolean arm, toggleFoundation;
        double hSlide, vSlide;
        boolean slowIntake;
        double slowwwwIntake;
        resetEncoders();

        while (opModeIsActive()) {
            //controller 1 stuff
            intakeIn = gamepad1.a && notInitController();
            intakeOut = gamepad1.x;
            intakeStop = gamepad1.b && notInitController();
            leftPower = actualControl(gamepad1.left_stick_x, 0.4) * .8;
            forwardPower = actualControl(gamepad1.left_stick_y, 0.5) * .9;
            rotate = actualControl(gamepad1.right_stick_x, .3);
            driveSlow = gamepad1.left_trigger;
            //controller 2 stuff
            arm = gamepad2.a && notInitController();
            toggleFoundation = gamepad2.y;
            slideSlow = gamepad2.left_trigger;
            vSlide = -gamepad2.left_stick_y;
            hSlide = gamepad2.right_stick_x;
            telemetry.addData("foundation state", foundationState);

            slowwwwIntake = gamepad1.right_trigger;


            //timer thingy
            telemetry.addData("slides", getVSlidePos());
            telemetry.addData("left slide", vLeft.getPower());
            telemetry.addData("right slide", vRight.getPower());
            slowIntake = slowwwwIntake > 0;
            if (controller1Timer.milliseconds() >= miliTillReady) {
                if (intakeIn) {
                    controller1Timer.reset();
                    inState = IntakeState.IN;
                }
                if (intakeOut) {
                    controller1Timer.reset();
                    inState = IntakeState.OUT;
                }
                if (intakeStop) {
                    controller1Timer.reset();
                    inState = IntakeState.STOP;
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
            //Motor Intake Control------------------------------------------------------------------
            telemetry.addData("state", inState);
            switch (inState) {
                case IN:
                    if (!slowIntake)
                        controlIntakeMotors(-0.5);
                    else
                        controlIntakeMotors(-0.2);
                    changeColor(RevBlinkinLedDriver.BlinkinPattern.DARK_BLUE);
                    break;
                case OUT:
                    if (!slowIntake)
                        controlIntakeMotors(0.3);
                    else
                        controlIntakeMotors(0.2);
                    changeColor(RevBlinkinLedDriver.
                            BlinkinPattern.YELLOW);
                    break;
                case STOP:
                    controlIntakeMotors(0);
                    changeColor(RevBlinkinLedDriver.BlinkinPattern.BLACK);
                    break;
            }
            //Speed Control-------------------------------------------------------------------------
            if (driveSlow > 0) {
                telemetry.addData("SLOW MODE ACTIVATED", " ");
                factor = 0.258;
            } else {
                telemetry.addData("NORMAL MODE", " ");
                factor = 0.87535463;
            }


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

                    switch (armState) {
                        case DROP:
                            grabServo(1);
                            break;
                        case PICK:
                            grabServo(0);
                            break;
                    }

                    if (slideSlow > 0)
                        slideFactor = 0.4;
                    else
                        slideFactor = 1;
                }

                //Arm Control---------------------------------------------------------------------------
                controlArm(hSlide);
                controlSlides(vSlide * slideFactor);

                telemetry.update();
            }
        }
    }
}
