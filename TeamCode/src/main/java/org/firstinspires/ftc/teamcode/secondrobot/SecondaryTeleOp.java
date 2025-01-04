package org.firstinspires.ftc.teamcode.secondrobot;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.secondrobot.horizontalslide.HorizontalIRollActions;
import org.firstinspires.ftc.teamcode.secondrobot.horizontalslide.HorizontalIntakeActions;
import org.firstinspires.ftc.teamcode.secondrobot.horizontalslide.HorizontalWristActions;
import org.firstinspires.ftc.teamcode.secondrobot.verticalslide.VerticalGrabberActions;
import org.firstinspires.ftc.teamcode.secondrobot.verticalslide.VerticalSlideActions;
import org.firstinspires.ftc.teamcode.secondrobot.verticalslide.VerticalWristActions;
import org.firstinspires.ftc.teamcode.secondrobot.horizontalslide.HorizontalSlideActions;
@TeleOp(name = "Tele Op second robot", group = "Linear Opmode")
public class SecondaryTeleOp extends HelperActions {
    private DriveActions driveActions = null;
    private HorizontalWristActions horizontalWrist = null;
    private HorizontalIntakeActions horizontalIntake = null;
    private HorizontalIRollActions horizontalIRoll = null;
    private VerticalSlideActions verticalSlide = null;
    private VerticalWristActions verticalWrist = null;
    private VerticalGrabberActions verticalGrabber = null;
    private HorizontalSlideActions horizontalSlide = null;

    double liftSpdMult = 0.8 ;

    @Override
    public void runOpMode() {
        driveActions = new DriveActions(telemetry, hardwareMap);
        horizontalSlide = new HorizontalSlideActions(hardwareMap,telemetry);
        horizontalWrist = new HorizontalWristActions(telemetry, hardwareMap);
        horizontalIntake = new HorizontalIntakeActions(telemetry, hardwareMap);
        horizontalIRoll = new HorizontalIRollActions(telemetry, hardwareMap);
        verticalSlide = new VerticalSlideActions(hardwareMap, telemetry);
        verticalWrist = new VerticalWristActions(telemetry, hardwareMap);
        verticalGrabber = new VerticalGrabberActions(telemetry, hardwareMap);
        //Set Speed for teleOp. Mecannum wheel speed.
        //driveActions.setSpeed(1.0);

        telemetry.addData("reverse speed?", "press down");
        telemetry.addData("normal speed?", "press up");
        telemetry.update();
        while (!(gamepad1.dpad_up || gamepad1.dpad_down));
        if (gamepad1.dpad_down) {
            setReverseSpeed(true);
        }
        telemetry.addData("registered", "you may begin");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        while (opModeIsActive()) {
            /** Gamepad 1 **/

            driveActions.drive(
                    //joystick controlling strafe
                    (gamepad1.left_stick_x * Math.abs(gamepad1.left_stick_x) * getReverseSpeed()),
                    //joystick controlling forward/backward
                    (-gamepad1.left_stick_y * Math.abs(gamepad1.left_stick_y) * getReverseSpeed()),
                    //joystick controlling rotation
                    gamepad1.right_stick_x * Math.abs(gamepad1.right_stick_x));
            telemetry.addData("Left stick x", gamepad1.left_stick_x);
            telemetry.addData("left stick y", gamepad1.left_stick_y);
            telemetry.addData("right stick x", gamepad1.right_stick_x);

            telemetry.addData("Joystick", gamepad2.left_stick_y);

            //allow you to use a d-pad to adjust the speed of the drive train
            changeSpeed(driveActions, gamepad1.dpad_up, gamepad1.dpad_down, false, false, gamepad1.right_trigger);
            toggleSpeed(gamepad1.a);


            /** Gamepad 2 **/
            //use the player 2 left joystick to run the horizontal slide
            horizontalSlide.teleOpArmMotor(-gamepad2.left_stick_y * Math.abs(gamepad2.left_stick_y), 1.2);
            //rotate the servo with intake in it
            horizontalWrist.flipping(gamepad2.left_bumper);
            //force the servo to flip bypassing the range limit apply
            horizontalWrist.override(gamepad1.b || gamepad2.b);
            //right trigger make the servo negative and left make the servo postivie.
            horizontalIntake.teleop(gamepad2.right_trigger > 0.05);
            //use the player two left joystick to run the horizonal roller
            horizontalIRoll.teleOp(gamepad2.y);

            //this assign the dpad to the diffrent level of the basket on the slide. left-bottom bar, down-bottom basket, right-top bar, up-top basket
            verticalSlide.goToPreset(gamepad2.dpad_left, gamepad2.dpad_down, gamepad2.dpad_right, gamepad2.dpad_up);
            //manually moves the vertical slide
            verticalSlide.teleOpVerticalSlide(-gamepad2.right_stick_y, 1);
            verticalSlide.turnOffAtBottom();
            //this set up a vertical wrist servo to down or up in a toggle way.
            verticalWrist.flipping(gamepad2.right_bumper);
            //vertical grabber servo, x-open, x-close
            verticalGrabber.teleOp(gamepad2.x || gamepad2.left_trigger > 0.05);
            //manages interface between different pieces of the exchange assembly
            updateExchangeAssembly(verticalGrabber, verticalWrist, horizontalWrist, horizontalSlide, verticalSlide, horizontalIRoll, horizontalIntake);

            //A button gamepad 2. Not yet working
//            managePlaceSample(gamepad2.a, verticalGrabber, verticalWrist, verticalSlide, horizontalWrist, horizontalSlide, horizontalIntake);

            telemetry.update();
        }
    }
}
