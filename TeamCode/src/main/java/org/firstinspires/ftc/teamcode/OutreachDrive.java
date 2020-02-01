package org.firstinspires.ftc.teamcode;

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
        IntakeState inState = IntakeState.STOP;
        FoundationState foundationState = FoundationState.RELEASE;
        ArmState armState = ArmState.REST;
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
            boolean slow = gamepad2.left_bumper;
            double vSlide = gamepad2.left_stick_y;
            double hSlide = gamepad2.right_stick_x;

            //timer thingy
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
            //Servo Intake Control------------------------------------------------------------------
            telemetry.addData("state", inState);
            switch (inState) {
                case IN:
                    controlIntakeMotors(0.9);
                    break;
                case OUT:
                    controlIntakeMotors(-0.5);
                    break;
                case STOP:
                    controlIntakeMotors(0);
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

                    switch (armState) {
                        case DROP:
                            grabServo(0.6);
                            break;
                        case PICK:
                            grabServo(0.2);
                            break;
                    }

                    if (slow)
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
