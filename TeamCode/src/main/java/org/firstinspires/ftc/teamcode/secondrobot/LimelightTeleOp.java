package org.firstinspires.ftc.teamcode.secondrobot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Configuration.secondRobot.HorizontalGrabberRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.HorizontalRollRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.HorizontalSlideRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.HorizontalWristRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.VerticalGrabberRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.VerticalHangerRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.VerticalSlideRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.VerticalWristRR;
import org.firstinspires.ftc.teamcode.FieldCentric.Attachment;
import org.firstinspires.ftc.teamcode.FieldCentric.DriveTrain;
import org.firstinspires.ftc.teamcode.GlobalVariables;
import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.secondrobot.horizontalslide.HorizontalIRollActions;
import org.firstinspires.ftc.teamcode.secondrobot.horizontalslide.HorizontalIntakeActions;
import org.firstinspires.ftc.teamcode.secondrobot.horizontalslide.HorizontalSlideActions;
import org.firstinspires.ftc.teamcode.secondrobot.horizontalslide.HorizontalWristActions;
import org.firstinspires.ftc.teamcode.secondrobot.verticalslide.VerticalGrabberActions;
import org.firstinspires.ftc.teamcode.secondrobot.verticalslide.VerticalHangerActions;
import org.firstinspires.ftc.teamcode.secondrobot.verticalslide.VerticalSlideActions;
import org.firstinspires.ftc.teamcode.secondrobot.verticalslide.VerticalWristActions;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "Limelight TeleOp", group = "Linear Opmode")
public class LimelightTeleOp extends HelperActions {
    private DriveActions driveActions = null;
    private HorizontalWristActions horizontalWrist = null;
    private HorizontalIntakeActions horizontalIntake = null;
    private HorizontalIRollActions horizontalIRoll = null;
    private VerticalSlideActions verticalSlide = null;
    private VerticalWristActions verticalWrist = null;
    private VerticalGrabberActions verticalGrabber = null;
    private VerticalHangerActions verticalHanger = null;
    private HorizontalSlideActions horizontalSlide = null;
    private LimeSweet limeSweet = null;
    PinpointDrive drive;
    DriveTrain driveTrain; Pose2d currentPose; Attachment attachment;
    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();
    VerticalSlideRR verticalSlideRR; VerticalWristRR verticalWristRR; VerticalGrabberRR verticalGrabberRR; VerticalHangerRR verticalHangerRR;
    HorizontalSlideRR horizontalSlideRR; HorizontalRollRR horizontalRollRR; HorizontalGrabberRR horizontalGrabberRR; HorizontalWristRR horizontalWristRR;


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
        verticalHanger = new VerticalHangerActions(hardwareMap);
        limeSweet = new LimeSweet(hardwareMap, telemetry, 0);
        //Set Speed for teleOp. Mecannum wheel speed.
        //driveActions.setSpeed(1.0);

        if(GlobalVariables.autoStarted){
            this.currentPose = GlobalVariables.currentPose;
            this.drive = new PinpointDrive(hardwareMap,currentPose);
            GlobalVariables.autoStarted = false;
        } else {
            this.currentPose = new Pose2d(-15, -64, Math.toRadians(180));
            this.drive = new PinpointDrive(hardwareMap, currentPose);
        }
        verticalSlideRR = new VerticalSlideRR(hardwareMap);
        verticalWristRR = new VerticalWristRR(hardwareMap , true);
        verticalGrabberRR = new VerticalGrabberRR(hardwareMap);

        horizontalSlideRR = new HorizontalSlideRR(hardwareMap);
        horizontalRollRR = new HorizontalRollRR(hardwareMap);
        horizontalGrabberRR = new HorizontalGrabberRR(hardwareMap);
        horizontalWristRR = new HorizontalWristRR(hardwareMap);
        verticalHangerRR = new VerticalHangerRR(hardwareMap);
        attachment = new Attachment(verticalSlideRR, verticalWristRR, verticalGrabberRR,
                horizontalSlideRR, horizontalRollRR, horizontalGrabberRR, horizontalWristRR, verticalHangerRR,
                runningActions, dash, drive);

        telemetry.addData("reverse speed?", "press down");
        telemetry.addData("normal speed?", "press up");
        telemetry.update();
        while (!(gamepad1.dpad_up || gamepad1.dpad_down) && opModeInInit());
        if (gamepad1.dpad_up) {
            setReverseSpeed(false);
        }
        telemetry.addData("registered", "you may begin");
        telemetry.update();
        boolean doSetExposure = true;
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        while (opModeIsActive()) {
//            if (doSetExposure) {
//                detectBlockActions.setExposure();
//                doSetExposure = false;
//            }
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
//            toggleSpeed(gamepad1.a);
            //toggles the hanger in and out, gamepad 1 x
            verticalHanger.teleOpHanger(gamepad1.x);
            //B button, overrides the coded stops on the vertical slide, resets slide perceived position on release
//            verticalSlide.manualResetSlides(gamepad1.b);
            verticalSlide.resetSlides(gamepad1.b);


            /** Gamepad 2 **/
            //use the player 2 left joystick to run the horizontal slide
            horizontalSlide.teleOpArmMotor(-gamepad2.left_stick_y * Math.abs(gamepad2.left_stick_y), 1.1);
            //If the right dpad button is pressed, hslide will toggle between 75% and 0%
            horizontalSlide.toggleSlidePosition(gamepad2.dpad_right);
            //rotate the servo with intake in it
            horizontalWrist.flipping(gamepad2.left_bumper);
            //force the servo to flip bypassing the range limit apply
            horizontalWrist.override(gamepad1.b || gamepad2.b);
            //right trigger make the servo negative and left make the servo positive.
            horizontalIntake.teleop(gamepad2.right_trigger > 0.05);
            //use the player two left joystick to run the horizonal roller
            horizontalIRoll.teleOp(gamepad2.y);

            //this assign the dpad to the diffrent level of the basket on the slide. left-bottom bar, down-bottom basket, right-top bar, up-top basket
            verticalSlide.goToPreset(gamepad2.dpad_left, gamepad2.dpad_down, false, gamepad2.dpad_up);
            //manually moves the vertical slide
            verticalSlide.teleOpVerticalSlide(-gamepad2.right_stick_y, 1);
//            verticalSlide.resetSlides(gamepad2.share);
            verticalSlide.turnOffAtBottom();
            //this set up a vertical wrist servo to down or up in a toggle way.
            verticalWrist.flipping(gamepad2.right_bumper);
            //vertical grabber servo, x-open, x-close
            verticalGrabber.teleOp(gamepad2.x || gamepad2.left_trigger > 0.05);
            //manages interface between different pieces of the exchange assembly
            updateExchangeAssembly(verticalGrabber, verticalWrist, horizontalWrist, horizontalSlide, verticalSlide, horizontalIRoll, horizontalIntake);

            //A button gamepad 2. Not yet working
            managePlaceSample(gamepad2.a, verticalGrabber, verticalWrist, verticalSlide, horizontalWrist, horizontalSlide, horizontalIntake, horizontalIRoll);

            manageGrabSample(gamepad2.share, limeSweet, horizontalSlide, driveActions, horizontalIRoll, horizontalWrist, horizontalIntake);
            limeSweet.scanButter();

            attachment.driveBasket(gamepad1.cross);
            attachment.updateAction();
//            Point blockCenter = detectBlockActions.pixelToPosition(detectBlockActions.getCenter());
//            telemetry.addData("block x %f, block y %f", blockCenter);
//            if (gamepad2.share) {
//                detectBlockActions.setExposure();
////                moveToBlock(detectBlockActions, driveActions, horizontalSlide, horizontalIRoll);
//            }

            telemetry.update();
        }
    }
}
