package org.firstinspires.ftc.teamcode;

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
        while (opModeIsActive()) {
            telemetry.addData("foundation state", foundationState);
            //controller 1 stuff
            boolean toggleIntake = gamepad1.a && notInitController();
            boolean stopIntake = gamepad1.b && notInitController();
            boolean toggleFoundation = gamepad1.y;
            double leftPower = actualControl(gamepad1.left_stick_x,0.4 ) * .5;
            double forwardPower = actualControl(gamepad1.left_stick_y, 0.5) * .7;
            double rotate = actualControl(gamepad1.right_stick_x, .3);


            //controller 2 stuff
            boolean arm = gamepad2.b && notInitController();
            boolean slideDown = gamepad2.b && notInitController();
            boolean slideUp = gamepad2.y;
            boolean slow = gamepad2.left_bumper;
            double vSlide = gamepad2.left_stick_y;
            double hSlide = gamepad2.right_stick_x;
            double swivelRight = gamepad2.right_trigger;
            double swivelLeft = -gamepad2.left_trigger;


            //timer thingy
            if(controller1Timer.milliseconds() >= miliTillReady) {
                if (toggleIntake) {
                    controller1Timer.reset();
                    if (inState.equals(IntakeState.IN)) {
                        inState = IntakeState.OUT;
                    } else {
                        inState = IntakeState.IN;
                    }
                }
                if (stopIntake) {
                    controller1Timer.reset();
                    inState = IntakeState.STOP;
                }
                if(toggleFoundation) {
                    controller1Timer.reset();
                    if(foundationState.equals(FoundationState.RELEASE)) {
                        foundationState = FoundationState.DRAG;
                        controlFoundation(foundationState);
                    } else{
                        foundationState = FoundationState.RELEASE;
                        controlFoundation(foundationState);
                    }
                }
            }
            //Servo Intake Control------------------------------------------------------------------
            telemetry.addData("state",inState);
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
           if(gamepad1.left_trigger > 0) {
               factor = 0.258;
           } else {
               factor = 0.87535463 ;
           }

            telemetry.addData("factor", factor);
            //Driving-------------------------------------------------------------------------------
            manDriveMotors(forwardPower, leftPower, rotate, factor);

            //timer thingy
            if(controller2Timer.milliseconds() >= miliTillReady) {
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
                }

                if(slideDown) {
                    moveSlides(-1);
                } else if (slideUp) {
                    moveSlides(1);
                }

                if(slow)
                    slideFactor = 0.4;
                else
                    slideFactor = 1;
            }

            //Arm Control---------------------------------------------------------------------------
            controlArm(hSlide);
            controlSlides(vSlide * slideFactor);
            swivelServo(swivelLeft + swivelRight);

            telemetry.update();
        }
    }
}
