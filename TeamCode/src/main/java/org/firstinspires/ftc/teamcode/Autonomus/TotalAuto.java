package org.firstinspires.ftc.teamcode.Autonomus;

// RR-specific imports

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.robotverticalslides.constants.ConfigConstants;

import java.util.Arrays;

@Config
@Autonomous(name = "Total Auto", group = "Autonomous")
public class TotalAuto extends LinearOpMode {
    private VerticalSlideRR verticalSlideRR = null;
    private HorizontalSlideRR horizontalSlideRR = null;
    private VerticalGrabberRR verticalGrabberRR = null;
    private VerticalWristRR verticalWristRR = null;
    private HorizontalWristRR horizontalWristRR = null;
    private HorizontalGrabberRR horizontalGrabberRR = null;

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
        public class Liftup implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                int position = -480;
                double velocity = 1800;
                if (!initialized) {
                    verticalSlide1.setTargetPosition(position);
                    verticalSlide1.setVelocity(velocity);
                    verticalSlide2.setTargetPosition(-position);
                    verticalSlide2.setVelocity(-velocity);
                    initialized = true;
                }
                double currentPosition = verticalSlide1.getCurrentPosition();
                if (currentPosition > position) {
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
                double velocity = 1800;
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
        private double open = Configuration.open;
        private double close = Configuration.close;
        public VerticalGrabberRR(Telemetry opModeTelemetry, HardwareMap opModeHardware) {
            this.telemetry = opModeTelemetry;
            this.hardwareMap = opModeHardware;
            verticalGrabberServo = hardwareMap.get(Servo.class, ConfigConstants.VERTICAL_GRABBER);
            verticalGrabberServo.setPosition(1.0);
        }
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
        double forwardUp = Configuration.forwardUp;
        double forwardDown = Configuration.forwardDown;
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
        public class HumanPickUp implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                verticalWristServo.setPosition(forwardUp);
                sleep(500);
                return false;
            }
        }
        public Action humanPickUp(){
            return new HumanPickUp();
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
    public class HorizontalWristRR{
        public Servo horizontalWristServo;
        private Telemetry telemetry;
        private HardwareMap hardwareMap;
        private double backwardPosIn = Configuration.backwardPosIn;
        private double backwardPosOut = Configuration.backwardPosOut;
        private double forwardPosOut = Configuration.forwardPosOut;
        public HorizontalWristRR(Telemetry opModeTelemetry, HardwareMap opModeHardware) {
            this.telemetry = opModeTelemetry;
            this.hardwareMap = opModeHardware;
            horizontalWristServo = hardwareMap.get(Servo.class, ConfigConstants.HORIZONTAL_WRIST);
            horizontalWristServo.setPosition(backwardPosIn);
        }
        public class GrabButter implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                horizontalWristServo.setPosition(forwardPosOut);
                return false;
            }
        }
        public Action grabButter(){
            return new GrabButter();
        }
        public class InRobot implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                horizontalWristServo.setPosition(backwardPosIn);
                return false;
            }
        }
        public Action inRobot(){
            return new InRobot();
        }
    }
    //HorizontalGrabber class
    public class HorizontalGrabberRR{
        public Servo intakeServo;
        private Telemetry telemetry;
        private HardwareMap hardwareMap;
        public HorizontalGrabberRR(Telemetry opModeTelemetry, HardwareMap opModeHardware) {
            this.telemetry = opModeTelemetry;
            this.hardwareMap = opModeHardware;
            intakeServo = hardwareMap.get(Servo.class, ConfigConstants.HORIZONTAL_INTAKE);
            intakeServo.setPosition(0);
        }
        private double floorClose = Configuration.floorClose;
        private double floorOpen = Configuration.floorOpen;
        public class FloorClose implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                intakeServo.setPosition(floorClose);
                return false;
            }
        }
        public Action floorClose(){
            return new FloorClose();
        }
        public class FloorOpen implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                intakeServo.setPosition(floorOpen);
                return false;
            }
        }
        public Action floorOpen(){
            return new FloorOpen();
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        //instantiate the robot to a particular pose.
        verticalSlideRR = new VerticalSlideRR(hardwareMap);
        horizontalSlideRR = new HorizontalSlideRR(hardwareMap, telemetry);
        verticalGrabberRR = new VerticalGrabberRR(telemetry, hardwareMap);
        verticalWristRR = new VerticalWristRR(telemetry, hardwareMap);
        horizontalWristRR = new HorizontalWristRR(telemetry, hardwareMap);
        horizontalGrabberRR = new HorizontalGrabberRR(telemetry, hardwareMap);

        //todo find the correct initial position and put it below
        Pose2d initialPose = new Pose2d(0,0, Math.toRadians(90));
        Vector2d vector2d = new Vector2d(0,0);
        PinpointDrive drive = new PinpointDrive(hardwareMap, initialPose);
        VelConstraint pushBlockVelOverride = new TranslationalVelConstraint(30);
        AccelConstraint pushBlockAccelOverride = new ProfileAccelConstraint(-10, 25);

        VelConstraint parkVelOverride = new TranslationalVelConstraint(80);
        VelConstraint parkAngularOverride = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(60),
                new AngularVelConstraint(Math.toRadians(90))
        ));
        AccelConstraint parkAccelOverride = new ProfileAccelConstraint(-50, 50);

        VelConstraint humanVelOverride = new TranslationalVelConstraint(30);
        AccelConstraint humanAccelOverride = new ProfileAccelConstraint(-7, 50);

        TrajectoryActionBuilder park = drive.actionBuilder(initialPose)
                .waitSeconds(.25)
                .strafeTo(new Vector2d(-10, 30), pushBlockVelOverride, pushBlockAccelOverride)
                .afterDisp(0, verticalGrabberRR.openGrabber())
                .waitSeconds(.2)
                .afterDisp(2, verticalSlideRR.setDown())
                .afterDisp(2, verticalWristRR.takeButter())
                .strafeTo(new Vector2d(-10, 15), parkVelOverride, parkAccelOverride)
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(26, 20,Math.toRadians(-90)),(-1)*Math.toRadians(90), parkAngularOverride, parkAccelOverride)
                .strafeTo(new Vector2d(27, 60), parkVelOverride, parkAccelOverride)
                .strafeTo(new Vector2d(36, 60), parkAngularOverride, parkAccelOverride)
                .strafeTo(new Vector2d(36, 18), humanVelOverride, humanAccelOverride)
                .strafeTo(new Vector2d(36, 30), parkVelOverride, parkAccelOverride)
                .waitSeconds(3)
                .afterDisp(0.0, verticalWristRR.wallButter())
                .strafeTo(new Vector2d(36, 18), humanVelOverride, humanAccelOverride)
                .afterTime(0.0, verticalGrabberRR.closeGrabber())
                .afterTime(0.0, verticalWristRR.wallButter())
                .waitSeconds(0.5)
                .strafeTo(new Vector2d(36, 30), parkVelOverride,parkAccelOverride)
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(0, 12,Math.toRadians(90)),Math.toRadians(90), parkAngularOverride, parkAccelOverride)
                .afterTime(0.0, verticalSlideRR.liftUp())
                .afterTime(0.0, verticalWristRR.wallButter())
                .waitSeconds(.25)
                .strafeTo(new Vector2d(-7, 31), pushBlockVelOverride, pushBlockAccelOverride)
                .afterDisp(0, verticalGrabberRR.openGrabber())
                .waitSeconds(.2)
                .afterDisp(2, verticalSlideRR.setDown())
                .afterDisp(2, verticalWristRR.takeButter())
                .strafeTo(new Vector2d(30, 8), parkVelOverride, parkAccelOverride);

        TrajectoryActionBuilder wait = drive.actionBuilder(initialPose)
                .waitSeconds(.5);

        TrajectoryActionBuilder hang = drive.actionBuilder(initialPose)
                .afterTime(0.0,verticalSlideRR.liftUp())
                .afterTime(0.0,verticalWristRR.wallButter())
                .waitSeconds(.25)
                .strafeTo(new Vector2d(-10, 30), pushBlockVelOverride, pushBlockAccelOverride)
                .afterDisp(0, verticalGrabberRR.openGrabber())
                .waitSeconds(.2)
                .afterDisp(2, verticalSlideRR.setDown())
                .afterDisp(2, verticalWristRR.takeButter())
                .strafeTo(new Vector2d(-10, 15));

        TrajectoryActionBuilder butter = drive.actionBuilder(initialPose)
                ;



        //initialize the robot
        Actions.runBlocking(
                new SequentialAction(
                        verticalGrabberRR.closeGrabber(),
                        verticalWristRR.takeButter(),
                        horizontalSlideRR.retractSlide(),
                        horizontalWristRR.inRobot(),
                        horizontalGrabberRR.floorClose()
                )
        );

        //ask the driver which auto they want to run
        boolean pickRightSide = false;
        boolean pickLeftSide = false;
        boolean pickHang = false;
        boolean pickHuman = false;
        boolean pickBasket = false;
        boolean pickPark = false;
        while(gamepad1.a =false) {
            telemetry.clear();
            telemetry.addLine("Which side?");
            telemetry.update();
            if (gamepad1.dpad_left == true && gamepad2.dpad_right == false) {
                pickLeftSide = true;
                while(!gamepad1.a){
                    telemetry.clear();
                    telemetry.addLine("D-Pad Left-hang: " + pickHang);
                    telemetry.addLine("D-Pad Up-Basket" + pickBasket);
                    telemetry.addLine("D-Pad Right-Park" + pickPark);
                    if(gamepad1.dpad_left == true && gamepad1.dpad_up == false && gamepad1.dpad_right == false && gamepad1.dpad_down == false){
                        pickHang = true;
                    }
                    if(gamepad1.dpad_left == false && gamepad1.dpad_up == true && gamepad1.dpad_right == false && gamepad1.dpad_down == false){
                        pickBasket = true;
                    }
                    if(gamepad1.dpad_left == false && gamepad1.dpad_up == false && gamepad1.dpad_right == true && gamepad1.dpad_down == false){
                        pickPark = true;
                    }
                    telemetry.update();
                }
            }
            if(gamepad1.dpad_left == false && gamepad1.dpad_right == true){
                pickRightSide = true;
                while(!gamepad1.a) {
                    telemetry.clear();
                    telemetry.addLine("D-Pad Left-Hang: " + pickHang);
                    telemetry.addLine("D-Pad Up-Human: " + pickHuman);
                    telemetry.addLine("D-Pad Right-Park" + pickPark);
                    if(gamepad1.dpad_left == true && gamepad1.dpad_up == false && gamepad1.dpad_right == false && gamepad1.dpad_down == false) {
                        pickHang = true;
                    }
                    if(gamepad1.dpad_left == false && gamepad1.dpad_up == true && gamepad1.dpad_right == false && gamepad1.dpad_down == false) {
                        pickHuman = true;
                    }
                    if(gamepad1.dpad_left == false && gamepad1.dpad_up == false && gamepad1.dpad_right == true && gamepad1.dpad_down == false) {
                        pickPark = true;
                    }
                    telemetry.update();
                }
            }
        }


        //wait for the start button to be press
        waitForStart();
        //if stop button is press, automatically stop
        if (isStopRequested()) return;

        //choose the trajectory


        //run the chosen action blocking
        Actions.runBlocking(
            new SequentialAction(
                    verticalSlideRR.liftUp(),
                    verticalWristRR.wallButter(),
                    park.build(),
                    wait.build()
            )
        );
    }
}



