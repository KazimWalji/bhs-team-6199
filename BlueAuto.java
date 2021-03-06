package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;


/**
 * Created by maryjaneb  on 11/13/2016.
 *
 * nerverest ticks
 * 60 1680
 * 40 1120
 * 20 560
 *
 * monitor: 640 x 480
 *YES
 */
@Autonomous(name= "BlueAuto", group="Sky autonomous")
//@Disabled//comment out this line before using
public class BlueAuto extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftRear = null;
    private DcMotor rightRear = null;
    private ElapsedTime runTime = new ElapsedTime();
    private DcMotor armMotor = null;
    private Servo armServo = null;
    //0 means skystone, 1 means yellow stone
    //-1 for debug, but we can keep it like this because if it works, it should change to either 0 or 255
    private static int valMid = -1;
    private static int valLeft = -1;
    private static int valRight = -1;

    private static float rectHeight = .6f / 8f;
    private static float rectWidth = 1.5f / 8f;

    private static float offsetX = .2f / 8f;//changing this moves the three rects and the three circles left or right, range : (-2, 2) not inclusive
    private static float offsetY = 1 / 8f;//changing this moves the three rects and circles up or down, range: (-4, 4) not inclusive

    private static float[] midPos = {4f / 8f + offsetX, 4f / 8f + offsetY};//0 = col, 1 = row
    private static float[] leftPos = {2f / 8f + offsetX, 4f / 8f + offsetY};
    private static float[] rightPos = {6f / 8f + offsetX, 4f / 8f + offsetY};
    //moves all rectangles right or left by amount. units are in ratio to monitor

    private final int rows = 320;
    private final int cols = 240;

    OpenCvCamera phoneCam;
    BNO055IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam.openCameraDevice();//open camera
        phoneCam.setPipeline(new StageSwitchingPipeline());//different stages
        phoneCam.startStreaming(rows, cols, OpenCvCameraRotation.UPRIGHT);//display on RC
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.loggingEnabled = false;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        // make sure the gyro is calibrated before continuing


        leftFront = hardwareMap.get(DcMotor.class, "left_front");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");
        leftRear = hardwareMap.get(DcMotor.class, "left_rear");
        rightRear = hardwareMap.get(DcMotor.class, "right_rear");
        armMotor = hardwareMap.get(DcMotor.class, "arm_motor");
        armServo = hardwareMap.get(Servo.class, "servo_arm");
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        int currLiftPos = armMotor.getCurrentPosition();
        int[] pos = {currLiftPos, 175, 400, 1100, 1899, 2879, 3099, 1299, 1499, 1699};

        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        armServo.setPosition(.5);
        runtime.reset();
        while (opModeIsActive()) {
            telemetry.addData("Values", valLeft + "   " + valMid + "   " + valRight);
            telemetry.addData("Height", rows);
            telemetry.addData("Width", cols);
            if (valLeft == 0) {
                telemetry.addData("pos", "left");
                telemetry.update();
                encoder(1200, 1200, 1200, 1200, .3, 6);
                encoder(-100, 100, 100, -100, .3, 3);
                encoder(100, 100, 100, 100, .3, 3);
                armServo.setPosition(.8);
                sleep(1000);
                encoder(-310, -310, -310, -310, .3, 3);

                turnAngle(85, 93, -.3, -.3, .3, .3);
                encoder(1700, 1700, 1700, 1700, .4, 7);
                armServo.setPosition(.5);
                sleep(1000);
                encoder(-2700, -2700, -2700, -2700, .4, 7);
                turnAngle(-3, 3, .3, .3, -.3, -.3);
                encoder(480, 480, 480, 480, .2, 3);

                armServo.setPosition(.8);
                sleep(1000);
                encoder(-350, -350, -350, -350, .3, 3);

                turnAngle(85, 93, -.3, -.3, .3, .3);
                encoder(3000, 3000, 3000, 3000, .4, 7);
                armServo.setPosition(.5);
                sleep(1000);
                encoder(-500, -500, -500, -500, .4, 5);
                armMotor.setTargetPosition(pos[0]);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(.4);
                while (armMotor.isBusy() && opModeIsActive()) {

                }
                telemetry.addData("pos", armMotor.getCurrentPosition());
                telemetry.update();
                armMotor.setPower(0);
                sleep(10000);
            } else if (valMid == 0) {

                telemetry.addData("pos", "center");
                telemetry.update();

                encoder(1200, 1200, 1200, 1200, .3, 7);
                encoder(300, -300, -300, 300, .3, 3);
                encoder(100, 100, 100, 100, .1, 3);
                armServo.setPosition(.8);
                sleep(1000);
                encoder(-300, -300, -300, -300, .3, 3);

                turnAngle(85, 93, -.3, -.3, .3, .3);
                encoder(2000, 2000, 2000, 2000, .4, 7);
                armServo.setPosition(.5);
                sleep(1000);

                encoder(-2970, -2970, -2970, -2970, .4, 7);
                turnAngle(-3, 3, .3, .3, -.3, -.3);
                encoder(300, 300, 300, 300, .1, 3);

                armServo.setPosition(.8);
                sleep(1000);
                encoder(-400, -400, -400, -400, .3, 3);

                turnAngle(85, 93, -.3, -.3, .3, .3);
                encoder(3200, 3200, 3200, 3200, .4, 7);
                armServo.setPosition(.5);
                sleep(1000);
                encoder(-400, -400, -400, -400, .4, 5);
                armMotor.setTargetPosition(pos[0]);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(.4);
                while (armMotor.isBusy() && opModeIsActive()) {

                }
                telemetry.addData("pos", armMotor.getCurrentPosition());
                telemetry.update();
                armMotor.setPower(0);
                sleep(10000);

            } else if (valRight == 0) {
                telemetry.addData("pos", "right");
                telemetry.update();

                encoder(1100, 1100, 1100, 1100, .4, 5);
                encoder(700, -700, -700, 700, .4, 3);
                encoder(100, 100, 100, 100, .25, 3);
                armServo.setPosition(.8);
                sleep(1000);
                encoder(-300, -300, -300, -300, .3, 3);

                turnAngle(85, 93, -.3, -.3, .3, .3);
                encoder(2500, 2500, 2500, 2500, .4, 7);
                armServo.setPosition(.5);
                sleep(1000);

                encoder(-2900, -2900, -2900, -2900, .4, 7);
                turnAngle(-3, 3, .3, .3, -.3, -.3);


                encoder(600, 600, 600, 600, .3, 3);
                encoder(-600, -600, -600, -600, .3, 3);


                encoder(500, -500, -500, 500, .3, 3);
                encoder(700, 700, 700, 700, .1, 3);
                armServo.setPosition(.8);
                sleep(1000);
                encoder(-600, -600, -600, -600, .3, 3);

                turnAngle(85, 93, -.3, -.3, .3, .3);
                encoder(3400, 3400, 3400, 3400, .8, 7);
                armServo.setPosition(.5);
                sleep(1000);
                encoder(-400, -400, -400, -400, .4, 5);
                armMotor.setTargetPosition(pos[0]);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(.4);
                while (armMotor.isBusy() && opModeIsActive()) {

                }
                telemetry.addData("pos", armMotor.getCurrentPosition());
                telemetry.update();
                armMotor.setPower(0);
                sleep(10000);

            }
            telemetry.update();

        }
    }

    //detection pipeline
    static class StageSwitchingPipeline extends OpenCvPipeline {
        Mat yCbCrChan2Mat = new Mat();
        Mat thresholdMat = new Mat();
        Mat all = new Mat();
        List<MatOfPoint> contoursList = new ArrayList<>();

        enum Stage {//color difference. greyscale
            detection,//includes outlines
            THRESHOLD,//b&w
            RAW_IMAGE,//displays raw view
        }

        private Stage stageToRenderToViewport = Stage.detection;
        private Stage[] stages = Stage.values();

        @Override
        public void onViewportTapped() {
            /*
             * Note that this method is invoked from the UI thread
             * so whatever we do here, we must do quickly.
             */

            int currentStageNum = stageToRenderToViewport.ordinal();

            int nextStageNum = currentStageNum + 1;

            if (nextStageNum >= stages.length) {
                nextStageNum = 0;
            }

            stageToRenderToViewport = stages[nextStageNum];
        }

        @Override
        public Mat processFrame(Mat input) {
            contoursList.clear();
            /*
             * This pipeline finds the contours of yellow blobs such as the Gold Mineral
             * from the Rover Ruckus game.
             */

            //color diff cb.
            //lower cb = more blue = skystone = white
            //higher cb = less blue = yellow stone = grey
            Imgproc.cvtColor(input, yCbCrChan2Mat, Imgproc.COLOR_RGB2YCrCb);//converts rgb to ycrcb
            Core.extractChannel(yCbCrChan2Mat, yCbCrChan2Mat, 2);//takes cb difference and stores

            //b&w
            Imgproc.threshold(yCbCrChan2Mat, thresholdMat, 102, 255, Imgproc.THRESH_BINARY_INV);

            //outline/contour
            Imgproc.findContours(thresholdMat, contoursList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
            yCbCrChan2Mat.copyTo(all);//copies mat object
            //Imgproc.drawContours(all, contoursList, -1, new Scalar(255, 0, 0), 3, 8);//draws blue contours


            //get values from frame
            double[] pixMid = thresholdMat.get((int) (input.rows() * midPos[1]), (int) (input.cols() * midPos[0]));//gets value at circle
            valMid = (int) pixMid[0];

            double[] pixLeft = thresholdMat.get((int) (input.rows() * leftPos[1]), (int) (input.cols() * leftPos[0]));//gets value at circle
            valLeft = (int) pixLeft[0];

            double[] pixRight = thresholdMat.get((int) (input.rows() * rightPos[1]), (int) (input.cols() * rightPos[0]));//gets value at circle
            valRight = (int) pixRight[0];

            //create three points
            Point pointMid = new Point((int) (input.cols() * midPos[0]), (int) (input.rows() * midPos[1]));
            Point pointLeft = new Point((int) (input.cols() * leftPos[0]), (int) (input.rows() * leftPos[1]));
            Point pointRight = new Point((int) (input.cols() * rightPos[0]), (int) (input.rows() * rightPos[1]));

            //draw circles on those points
            Imgproc.circle(all, pointMid, 5, new Scalar(255, 0, 0), 1);//draws circle
            Imgproc.circle(all, pointLeft, 5, new Scalar(255, 0, 0), 1);//draws circle
            Imgproc.circle(all, pointRight, 5, new Scalar(255, 0, 0), 1);//draws circle

            //draw 3 rectangles
            Imgproc.rectangle(//1-3
                    all,
                    new Point(
                            input.cols() * (leftPos[0] - rectWidth / 2),
                            input.rows() * (leftPos[1] - rectHeight / 2)),
                    new Point(
                            input.cols() * (leftPos[0] + rectWidth / 2),
                            input.rows() * (leftPos[1] + rectHeight / 2)),
                    new Scalar(0, 255, 0), 3);
            Imgproc.rectangle(//3-5
                    all,
                    new Point(
                            input.cols() * (midPos[0] - rectWidth / 2),
                            input.rows() * (midPos[1] - rectHeight / 2)),
                    new Point(
                            input.cols() * (midPos[0] + rectWidth / 2),
                            input.rows() * (midPos[1] + rectHeight / 2)),
                    new Scalar(0, 255, 0), 3);
            Imgproc.rectangle(//5-7
                    all,
                    new Point(
                            input.cols() * (rightPos[0] - rectWidth / 2),
                            input.rows() * (rightPos[1] - rectHeight / 2)),
                    new Point(
                            input.cols() * (rightPos[0] + rectWidth / 2),
                            input.rows() * (rightPos[1] + rectHeight / 2)),
                    new Scalar(0, 255, 0), 3);

            switch (stageToRenderToViewport) {
                case THRESHOLD: {
                    return thresholdMat;
                }

                case detection: {
                    return all;
                }

                case RAW_IMAGE: {
                    return input;
                }

                default: {
                    return input;
                }
            }
        }
    }

    public void encoder(int lf, int rf, int lr, int rr, double pow, int sec) {
        pow = pow + .1;
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode((DcMotor.RunMode.STOP_AND_RESET_ENCODER));

        rightFront.setTargetPosition(rf);
        leftRear.setTargetPosition(lr);
        rightRear.setTargetPosition(rr);
        leftFront.setTargetPosition(lf);

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRear.setMode((DcMotor.RunMode.RUN_TO_POSITION));


        leftFront.setPower(pow);
        rightFront.setPower(pow);
        rightRear.setPower(pow);
        leftRear.setPower(pow);
        leftFront.setPower(pow);
        runTime.reset();
        while (rightRear.isBusy() || leftRear.isBusy() || rightFront.isBusy() | leftFront.isBusy() && opModeIsActive() && runTime.seconds() < sec) {


        }

        leftFront.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
        leftRear.setPower(0);
    }

    public void turnAngle(int firstA, int secA, double pow1, double pow2, double pow3, double pow4) {
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double currAngle;
        Orientation angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        currAngle = (angle.firstAngle + 360) % 360;
        telemetry.addData("currAngle", currAngle);
        telemetry.update();
        while ((currAngle < firstA || currAngle > secA) && opModeIsActive()) {
            leftFront.setPower(pow1);
            leftRear.setPower(pow2);
            rightRear.setPower(pow3);
            rightFront.setPower(pow4);
            angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            currAngle = (angle.firstAngle + 360) % 360;
            telemetry.addData("currAngle", currAngle);
            telemetry.update();
        }
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
        rightFront.setPower(0);

    }

    public double getHeading() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return (angles.firstAngle + 360) % 360;
    }
}