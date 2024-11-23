
package org.firstinspires.ftc.teamcode.robotverticalslides;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robotverticalslides.HorizontalSlide.HorizontalSlideActions;
import org.firstinspires.ftc.teamcode.robotverticalslides.HorizontalSlide.HorizontalWristActions;
import org.firstinspires.ftc.teamcode.robotverticalslides.HorizontalSlide.HorizontalIntakeActions;
import org.firstinspires.ftc.teamcode.robotverticalslides.VerticalSlide.VerticalGrabberActions;
import org.firstinspires.ftc.teamcode.robotverticalslides.VerticalSlide.VerticalSlideActions;
import org.firstinspires.ftc.teamcode.robotverticalslides.VerticalSlide.VerticalWristActions;
@Disabled
@TeleOp(name = "Manual Tele Op", group = "Linear Opmode")
public class ManualTeleOp extends HelperActions {

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
                    (gamepad1.left_stick_x * Math.abs(gamepad1.left_stick_x)),      //joystick controlling strafe
                    (-gamepad1.left_stick_y * Math.abs(gamepad1.left_stick_y)),     //joystick controlling forward/backward
                    driveStraight(gamepad1.right_stick_x));    //joystick controlling rotation
            telemetry.addData("Left stick x", gamepad1.left_stick_x);
            telemetry.addData("left stick y", gamepad1.left_stick_y);
            telemetry.addData("right stick x", gamepad1.right_stick_x);

            telemetry.addData("Joystick", gamepad2.left_stick_y);

            changeSpeed(driveActions, gamepad1.dpad_up, gamepad1.dpad_down, false, false, gamepad1.right_trigger);
            toggleSpeed(gamepad1.a);

            horizontalSlide.teleOpHorizontalSlide(-gamepad2.left_stick_y, 1);
            horizontalWrist.manual(gamepad2.left_bumper, gamepad2.b);
            horizontalIntake.teleop(gamepad2.right_trigger, gamepad2.left_trigger);

            verticalSlide.teleOpVerticalSlide(-gamepad2.right_stick_y, 1);
//            verticalSlide.maintainVerticalSlide2();
//            verticalSlide.setOnRung(gamepad2.a);
            verticalWrist.manual(gamepad2.right_bumper, gamepad2.b);
            verticalGrabber.teleOp(gamepad2.y, gamepad2.x);

//            updateExchangeAssembly(verticalGrabber, verticalWrist, horizontalWrist, horizontalSlide, verticalSlide);

            telemetry.update();
        }

//        close(verticalGrabber, verticalWrist, verticalSlide, horizontalWrist, horizontalSlide);
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
