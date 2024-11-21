package org.firstinspires.ftc.teamcode.Autonomus;

// RR-specific imports
import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

//Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

//Team code imports
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.robotverticalslides.DriveActions;
import org.firstinspires.ftc.teamcode.robotverticalslides.HorizontalSlide.HorizontalSlideActions;
import org.firstinspires.ftc.teamcode.robotverticalslides.HorizontalSlide.HorizontalWristActions;
import org.firstinspires.ftc.teamcode.robotverticalslides.HorizontalSlide.HorizontalIntakeActions;
import org.firstinspires.ftc.teamcode.robotverticalslides.VerticalSlide.VerticalGrabberActions;
import org.firstinspires.ftc.teamcode.robotverticalslides.VerticalSlide.VerticalSlideActions;
import org.firstinspires.ftc.teamcode.robotverticalslides.VerticalSlide.VerticalWristActions;
import org.firstinspires.ftc.teamcode.robotverticalslides.constants.ConfigConstants;
import org.firstinspires.ftc.teamcode.Autonomus.Configuration;


@Config
@Autonomous(name = "MainAuto", group = "Autonomus")
public class MainAutonomus extends LinearOpMode {

    private HorizontalSlideActions horizontalSlide = null;
    private HorizontalWristActions horizontalWrist = null;
    private HorizontalIntakeActions horizontalIntake = null;
    private VerticalWristActions verticalWrist = null;
    private VerticalSlideActions verticalSlide = null;
    private VerticalGrabberActions verticalGrabber = null;
    private VerticalSlideRR verticalSlideRR = null;
    private HorizontalSlideRR horizontalSlideRR = null;
    private VerticalGrabberRR verticalGrabberRR = null;
    private VerticalWristRR verticalWristRR = null;

    public class VerticalSlideRR{
        public DcMotorEx verticalSlide1 = null;
        public DcMotorEx verticalSlide2 = null;

        public VerticalSlideRR(HardwareMap hardwareMap){
            verticalSlide1 = hardwareMap.get(DcMotorEx.class, ConfigConstants.VERTICAL_SLIDE1);
            verticalSlide1.setDirection(DcMotorSimple.Direction.REVERSE);
            verticalSlide1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            verticalSlide1.setTargetPosition(0);
            verticalSlide1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            verticalSlide2 = hardwareMap.get(DcMotorEx.class, ConfigConstants.VERTICAL_SLIDE2);
            verticalSlide2.setDirection(DcMotorSimple.Direction.FORWARD);
            verticalSlide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            verticalSlide2.setTargetPosition(0);
            verticalSlide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        public class Liftup implements Action{
            private boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                int position = -480;
                double velocity = 1800;
                if(!initialized){
                        verticalSlide1.setTargetPosition(position);
                        verticalSlide1.setVelocity(velocity);
                        verticalSlide2.setTargetPosition(-position);
                        verticalSlide2.setVelocity(-velocity);
                    initialized = true;
                }
                double currentPosition = verticalSlide1.getCurrentPosition();
                if (currentPosition > position){
                    return true;
                } else {
                    return false;

                }
            }
        }
        public Action liftUp(){
            return new Liftup();
        }
        public class SetDown implements Action{
            private boolean initialized = false;
            public boolean run(@NonNull TelemetryPacket packet){
                int position = 0;
                double velocity = 1800;
                if(!initialized){
                    verticalSlide1.setTargetPosition(position);
                    verticalSlide1.setVelocity(velocity);
                    verticalSlide2.setTargetPosition(-position);
                    verticalSlide2.setVelocity(-velocity);
                    initialized = true;
                }
                double currentPosition = verticalSlide1.getCurrentPosition();
                if (currentPosition > position){
                    return true;
                } else {
                    return false;

                }
            }
        }
        public Action setDown(){
            return new SetDown();
        }
    }
    //make a class for horizontal Slide
    public class HorizontalSlideRR{
        public DcMotorEx HorizontalSlide2 = null;
        private Telemetry telemetry;
        public HorizontalSlideRR(HardwareMap hardwareMap, Telemetry telemetry) {
            this.telemetry = telemetry;
            HorizontalSlide2 = hardwareMap.get(DcMotorEx.class, ConfigConstants.HORIZONTAL_SLIDE2);
            HorizontalSlide2.setDirection(DcMotorSimple.Direction.FORWARD);
            HorizontalSlide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            HorizontalSlide2.setTargetPosition(0);
            HorizontalSlide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        private boolean initialized = false;
        public class RetractSlide implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                int position = 0;
                double velocity = 0;
                if(!initialized){
                    HorizontalSlide2.setTargetPosition(position);
                    HorizontalSlide2.setVelocity(velocity);
                    initialized = true;
                }
                double currentPosition = HorizontalSlide2.getCurrentPosition();
                if(currentPosition > position){
                    return true;
                }else {
                    return false;
                }
            }
        }
        public Action retractSlide(){
            return new RetractSlide();
        }
    }
    //add a class for the vertical grabber
    public class VerticalGrabberRR{
        public Servo verticalGrabberServo;
        private Telemetry telemetry;
        private HardwareMap hardwareMap;
        public VerticalGrabberRR(Telemetry opModeTelemetry, HardwareMap opModeHardware) {
            this.telemetry = opModeTelemetry;
            this.hardwareMap = opModeHardware;
            verticalGrabberServo = hardwareMap.get(Servo.class, ConfigConstants.VERTICAL_GRABBER);
            verticalGrabberServo.setPosition(1.0);
        }
        double close = 0.5;
        double open = 0.3;
        public class CloseGrabber implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                verticalGrabberServo.setPosition(close);
                return false;
            }
        }
        public Action closeGrabber(){
            return new CloseGrabber();
        }
        public class OpenGrabber implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                verticalGrabberServo.setPosition(open);
                return false;
            }
        }
        public Action openGrabber(){
            return new OpenGrabber();
        }
    }
    //add Vertical Wrist Class
    public class VerticalWristRR{
        public Servo verticalWristServo;
        private Telemetry telemetry;
        private HardwareMap hardwareMap;

        //this is a position to place it on the basket
        double forwardUp = Configuration.forwardUp;
        //this is a position to grab the butter from the wall or set it on the lower basket or either rung
        double forwardDown = Configuration.forwardDown;
        //this is the position to grab the butter from the intake
        double backwardPos = Configuration.backwardPos;
        public VerticalWristRR(Telemetry opModeTelemetry, HardwareMap opModeHardware) {
            this.telemetry = opModeTelemetry;
            this.hardwareMap = opModeHardware;
            verticalWristServo = hardwareMap.get(Servo.class, ConfigConstants.VERTICAL_WRIST);
            verticalWristServo.setPosition(backwardPos);
        }
        public class PlaceBasket implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                verticalWristServo.setPosition(forwardUp);
                return false;
            }
        }
        public Action placeBasket(){
            return new PlaceBasket();
        }
        public class WallButter implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                verticalWristServo.setPosition(forwardDown);
                return false;
            }
        }
        public Action wallButter(){
            return new WallButter();
        }
        public class TakeButter implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                verticalWristServo.setPosition(backwardPos);
                return false;
            }
        }
        public Action takeButter(){
            return new TakeButter();
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        //instantiate the robot to a particular pose.
        verticalSlideRR = new VerticalSlideRR(hardwareMap);
        horizontalSlideRR = new HorizontalSlideRR(hardwareMap, telemetry);
        verticalGrabberRR = new VerticalGrabberRR(telemetry, hardwareMap);
        verticalWristRR = new VerticalWristRR(telemetry, hardwareMap);

        //todo find the correct initial position and put it below
        Pose2d initialPose = new Pose2d(0,0, Math.toRadians(90));
        Vector2d vector2d = new Vector2d(0,0);
        PinpointDrive drive = new PinpointDrive(hardwareMap, initialPose);

        //trajectory from initial spot moving to blue parking spot
        TrajectoryActionBuilder testAuto = drive.actionBuilder(initialPose)
                .setTangent(0)
                .splineToConstantHeading(new Vector2d(-5,15),Math.toRadians(0));
        //todo
                new TranslationalVelConstraint(1);
                new ProfileAccelConstraint(-.5,.5);

        TrajectoryActionBuilder pushBlock = drive.actionBuilder(initialPose)
                .waitSeconds(1)
                .splineToConstantHeading(new Vector2d(-5, 25), 0);
        new TranslationalVelConstraint(1);
        new ProfileAccelConstraint(-.5,.5);

        TrajectoryActionBuilder resetPush = drive.actionBuilder(initialPose)
                .waitSeconds(.5)
                .splineToConstantHeading(new Vector2d(-5,5), 0)
                .waitSeconds(.5);

        TrajectoryActionBuilder park = drive.actionBuilder(initialPose)
                .splineToConstantHeading(new Vector2d(18,5), 0)
               .splineToConstantHeading(new Vector2d(18, 50), 0)
                .splineToConstantHeading(new Vector2d(28, 50), 0)
                .splineToConstantHeading(new Vector2d(28, 5), 0);
        new TranslationalVelConstraint(1);
        new ProfileAccelConstraint(-.5,.5);

        TrajectoryActionBuilder wait = drive.actionBuilder(initialPose)
            .waitSeconds(.5);

        TrajectoryActionBuilder parkBlock = drive.actionBuilder(initialPose)
                .splineToConstantHeading(new Vector2d(30, 40), 0);
//                .splineToConstantHeading(new Vector2d(42, 40), 0)
//                .splineToConstantHeading(new Vector2d(42, 5), 0);




        //initialize the robot
        Actions.runBlocking(
                new SequentialAction(
                        verticalGrabberRR.closeGrabber(),
                        verticalWristRR.takeButter(),
                        horizontalSlideRR.retractSlide()
                )
        );

        //wait for the start button to be press
        waitForStart();
        //if stop button is press, automatically stop
        if (isStopRequested()) return;

        //run the chosen action blocking
        Actions.runBlocking(
            new SequentialAction(
                    testAuto.build(),
                    verticalSlideRR.liftUp(),
                    verticalWristRR.wallButter(),
                    pushBlock.build(),
                    verticalGrabberRR.openGrabber(),
                    resetPush.build(),
                    verticalWristRR.takeButter(),
                    verticalSlideRR.setDown(),
                    verticalGrabberRR.closeGrabber(),
                    park.build()
            )
        );
    }
}



