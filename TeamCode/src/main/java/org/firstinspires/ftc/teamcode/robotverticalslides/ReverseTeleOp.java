
package org.firstinspires.ftc.teamcode.robotverticalslides;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robotverticalslides.HorizontalSlide.HorizontalIntakeActions;
import org.firstinspires.ftc.teamcode.robotverticalslides.HorizontalSlide.HorizontalSlideActions;
import org.firstinspires.ftc.teamcode.robotverticalslides.HorizontalSlide.HorizontalWristActions;
import org.firstinspires.ftc.teamcode.robotverticalslides.VerticalSlide.VerticalGrabberActions;
import org.firstinspires.ftc.teamcode.robotverticalslides.VerticalSlide.VerticalSlideActions;
import org.firstinspires.ftc.teamcode.robotverticalslides.VerticalSlide.VerticalWristActions;

@TeleOp(name = "Reverse Tele Op 2 Slide Robot", group = "Linear Opmode")
public class ReverseTeleOp extends HelperActions {

    private DriveActions driveActions = null;
    private HorizontalSlideActions horizontalSlide = null;
    private HorizontalWristActions horizontalWrist = null;
    private HorizontalIntakeActions horizontalIntake = null;
    private VerticalSlideActions verticalSlide = null;
    private VerticalWristActions verticalWrist = null;
    private VerticalGrabberActions verticalGrabber = null;

    boolean correctRotation = false;
    double rotationPosition = 0;
    double rotation = 0;
    double liftSpdMult = 0.8 ;

    @Override
    public void runOpMode() {

        driveActions = new DriveActions(telemetry, hardwareMap);
        horizontalSlide = new HorizontalSlideActions(hardwareMap, telemetry);
        horizontalWrist = new HorizontalWristActions(telemetry, hardwareMap);
        horizontalIntake = new HorizontalIntakeActions(telemetry, hardwareMap);
        verticalSlide = new VerticalSlideActions(hardwareMap, telemetry);
        verticalWrist = new VerticalWristActions(telemetry, hardwareMap);
        verticalGrabber = new VerticalGrabberActions(telemetry, hardwareMap);


        //Set Speed for teleOp. Mecannum wheel speed.
        //driveActions.setSpeed(1.0);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        while (opModeIsActive()) {

            /** Gamepad 1 **/

            driveActions.drive(
                    //joystick controlling strafe
                    (-gamepad1.left_stick_x * Math.abs(gamepad1.left_stick_x)),
                    //joystick controlling forward/backward
                    (gamepad1.left_stick_y * Math.abs(gamepad1.left_stick_y)),
                    //joystick controlling rotation
                    driveStraight(gamepad1.right_stick_x));
            telemetry.addData("Left stick x", gamepad1.left_stick_x);
            telemetry.addData("left stick y", gamepad1.left_stick_y);
            telemetry.addData("right stick x", gamepad1.right_stick_x);

            telemetry.addData("Joystick", gamepad2.left_stick_y);

            //allow you to use a d-pad to adjust the speed of the drive train
            changeSpeed(driveActions, gamepad1.dpad_up, gamepad1.dpad_down, false, false, gamepad1.right_trigger);
            toggleSpeed(gamepad1.a);
            
            /** Gamepad 2 **/
            //use the player 2 left joystick to run the horzontal slide
            horizontalSlide.teleOpHorizontalSlide(-gamepad2.left_stick_y, 1.8);
            //rotate the servo with intake in it
            horizontalWrist.flipping(gamepad2.left_bumper);
            //force the servo to flip bypassing the range limit apply
            horizontalWrist.override(gamepad1.b || gamepad2.b);
            //right trigger make the servo negative and left make the servo postivie.
            horizontalIntake.teleop(gamepad2.right_trigger > 0.05 || gamepad2.left_trigger > 0.05);

            //this assign the dpad to the diffrent level of the basket on the slide. left-bottom bar, down-bottom basket, right-top bar, up-top basket
            verticalSlide.goToPreset(gamepad2.dpad_left, gamepad2.dpad_down, gamepad2.dpad_right, gamepad2.dpad_up);
            //manually moves the vertical slide
            verticalSlide.teleOpVerticalSlide(gamepad2.right_stick_y, 1);
//            verticalSlide.maintainVerticalSlide2();
            verticalWrist.flipping(gamepad2.right_bumper);
            //this set up a vertical wrist servo to down or up in a toggle way.
            //vertical grabber servo, y-close, x-close
            verticalGrabber.teleOp(gamepad2.y, gamepad2.x);
            //manages interface between different pieces of the exchange assembly
            updateExchangeAssembly(verticalGrabber, verticalWrist, horizontalWrist, horizontalSlide, verticalSlide);

//            if (gamepad2.left_trigger > 0.05) {
//                placeSample(verticalGrabber, verticalWrist, verticalSlide, horizontalWrist, horizontalSlide, horizontalIntake);
//            } else{
//                resetPlaceState();
//            }
//            telemetry.update();
        }
        telemetry.addData("[ROBOTNAME] ", "Going");
        telemetry.update();
//        idle();

    }

    // Code to make it drive straight
    double gainMultiplier = 0.0005;
    private double driveStraight(double rightStickX) {
//        if(Math.abs(rightStickX) > 0.01){ // Only correct position when not rotating
//            rotation = rightStickX * Math.abs(rightStickX); // Rotating voluntarily
//            correctRotation = false;
//        } else if (!correctRotation){ // If not rotating, get the position rotationally once when the turn is done
//            if (!isTurning()) {
//                correctRotation = true;
//                rotationPosition = driveActions.getRawHeading();
//            }
//            rotation = 0;
//        } else { // Correct rotation when not turning
//            double gain = driveActions.leftFront.getVelocity() + driveActions.rightFront.getVelocity() + driveActions.leftRear.getVelocity() + driveActions.rightRear.getVelocity();
//            rotation = -driveActions.getSteeringCorrection(rotationPosition, gain * gainMultiplier);
//        }
//        return rotation;
        return  rightStickX * Math.abs(rightStickX);
    }
//    private boolean isTurning() {
//        return Math.abs((driveActions.leftFront.getVelocity() + driveActions.leftRear.getVelocity()) - (driveActions.rightFront.getVelocity() + driveActions.rightRear.getVelocity())) < 1;
//    }
}
