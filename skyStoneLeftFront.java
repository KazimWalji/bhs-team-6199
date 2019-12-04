/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.util.List;
import java.util.Locale;
/**
 * This 2019-2020 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the Skystone game elements.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@Autonomous(name = "SkyStoneLeftFront", group = "Concept")
//@Disabled
public class skyStoneLeftFront extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftRear = null;
    private DcMotor rightRear = null;
    private Servo armServo = null;
    private ElapsedTime runTime = new ElapsedTime();// dont have to reset timer every time
    private ElapsedTime runTime2 = new ElapsedTime();
    private int x;
    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "Aa21Mu//////AAABmfhDWd3BsU7Djc6+zx83U9UcxVIrPsep7RVbJV5Js1pYIhi48LApGw7Zmblx9PuGu7CnYS5BLybe70rZiXWTfrKXnzaN4C4A6clpD+MF6aJFKWZoxoczRmnEWE7XMiFq42nbiey2bV6m1ucyXtGVK7HCmVfRVrsJI2v4+Z8Vf1cAIevSq+MDwD7M0p1NhRaOPK29FqVypPeiqECwZeITOvFgcwSu4JA5DfILEqThKcHkS4N+1IYfwR1c9b35zaMuuOQxt+Qjmbu4jzILBvOiFFI7Y3bQk9lSqYe/IY1wGgxnC1KSLojMchnENel9KkDQNQ2A4XUpcIhk2p2WHIVT6OSmCcrVwXd2MQ8lvISoQj2H";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        leftFront = hardwareMap.get(DcMotor.class, "left_front");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");
        leftRear = hardwareMap.get(DcMotor.class, "left_rear");
        rightRear = hardwareMap.get(DcMotor.class, "right_rear");
        armServo = hardwareMap.get(Servo.class, "servo_arm");
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        x = 0;

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");//prints it out on phone
        }

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();
        armServo.setPosition(.4);

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if(x==0) {
                    encoder(500,500,500,500, .3, 3);
                    x++;
                }

                telemetry.addData("Encoders", "worked");
                telemetry.update();
                sleep(100);
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                      telemetry.addData("# Object Detected", updatedRecognitions.size());
                        telemetry.update();


                            int skyStone = -1;

                            for (Recognition recognition : updatedRecognitions) {
                                if (recognition.getLabel().equals("Skystone")) {
                                    telemetry.addData("left pos", recognition.getLeft());
                                    telemetry.update();
                                    skyStone = (int) recognition.getLeft();

                                }
                                else if (recognition.getLabel().equals("Stone"))
                                {
                                    telemetry.addData("It: ", "WORKS");
                                    telemetry.update();
                                }
                            }
                            if (skyStone != -1) {
                                if (skyStone < 75) {
                                    telemetry.addData("Gold Mineral Position", "Left");
                                    telemetry.update();

                                    //Strafes left
                                    encoder(-400,400,400,-400, .3, 3);

                                    //goes forward to get the skystone

                                    encoder(900,900,900,900, .3, 3);

                                    armServo.setPosition(.75);

                                    //arm is down, and goes backward with the stone

                                    encoder(-200,-200,-200,-200, .3, 3);

                                    //rotate's 90 degrees

                                    encoder(-914,914,-914,914, .3, 4);
                                    //goes forward to drop the stone on the foundation side

                                    encoder(2250,2250,2250,2250, .5, 6);

                                    armServo.setPosition(.4);

                                    //drops stone and goes backward all the way to the edge

                                    encoder(-2990,-2990,-2990,-2990, .5, 6);

                                    //rotates so its facing the stones again

                                    encoder(914,-914,914,-914, .3, 4);

                                    // robot goes back so it can see all three stones 2nd time

                                    encoder(-700,-700,-700,-700, .3, 4);


                                }
                                else if (skyStone > 300) {


                                    telemetry.addData("Gold Mineral Position", "Right");
                                    telemetry.addData("left pos", skyStone);
                                    telemetry.update();

                                    //Strafes right
                                    encoder(550,-550,-550,550, .3, 3);

                                    //goes forward to get the skystone

                                    encoder(950,950,950,950, .3, 4);

                                    armServo.setPosition(.75);

                                    //arm is down, and goes backward with the stone

                                    encoder(-250,-250,-250,-250, .3, 3);

                                    //rotate's 90 degrees

                                    encoder(-914,914,-914,914, .3, 4);

                                    //goes forward to drop the stone on the foundation side

                                    encoder(2200,2200,2200,2200, .5, 6);

                                    armServo.setPosition(.4);

                                    //drops stone and goes backward all the way to the edge

                                    encoder(-3000,-3000,-3000,-3000, .5, 6);

                                    //rotates so its facing the stones again

                                    encoder(914,-914,914,-914, .3, 4);

                                    // robot goes back so it can see all three stones 2nd time

                                    encoder(-700,-700,-700,-700, .3, 4);

                                }


                                else if (skyStone < 300 && skyStone >100)
                                {
                                        telemetry.addData("Gold Mineral Position", "Center");
                                    telemetry.addData("left pos", skyStone);
                                        telemetry.update();

                                    //goes forward to get the skystone

                                    encoder(800,800,800,800, .3, 4);

                                    armServo.setPosition(.75);

                                    //arm is down, and goes backward with the stone

                                    encoder(-270,-270,-270,-270, .3, 2);

                                    //rotate's 90 degrees

                                    encoder(-914,914,-914,914, .3, 4);

                                    //goes forward to drop the stone on the foundation side

                                    encoder(2250,2250,2250,2250, .3, 7);

                                    armServo.setPosition(.4);

                                    //drops stone and goes backward all the way to the edge

                                    encoder(-3000,-3000,-3000,-3000, .3, 6);

                                    //rotates so its facing the stones again

                                    encoder(918,-918,918,-918, .3, 4);

                                    // robot goes back so it can see all three stones 2nd time

                                    encoder(-550,-550,-550,-550, .3, 4);

                                    telemetry.update();
                            }
                            telemetry.update();
                        }
                        telemetry.update();
                    }
                    telemetry.update();
                }
            }
        }

        if (tfod != null) {
            tfod.shutdown();
        }
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
            "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.5;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
    public void encoder (int lf, int rf, int lr, int rr, double pow, int sec )
    {
        leftFront = hardwareMap.get(DcMotor.class, "left_front");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");
        leftRear = hardwareMap.get(DcMotor.class, "left_rear");
        rightRear = hardwareMap.get(DcMotor.class, "right_rear");
        armServo = hardwareMap.get(Servo.class, "servo_arm");
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode((DcMotor.RunMode.STOP_AND_RESET_ENCODER));

        leftFront.setTargetPosition(lf);
        rightFront.setTargetPosition(rf);
        leftRear.setTargetPosition(lr);
        rightRear.setTargetPosition(rr);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRear.setMode((DcMotor.RunMode.RUN_TO_POSITION));


        leftFront.setPower(pow);
        rightFront.setPower(pow);
        rightRear.setPower(pow);
        leftRear.setPower(pow);
        runTime.reset();
        while (leftFront.isBusy() && rightRear.isBusy() && leftRear.isBusy() && rightFront.isBusy() && opModeIsActive() && runTime.seconds() < sec) {

        }

        leftFront.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
        leftRear.setPower(0);
    }



}
