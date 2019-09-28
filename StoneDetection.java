package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import java.lang.annotation.Target;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaSkyStone;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TfodSkyStone;

@Autonomous(name = "SeekSkyStone (Blocks to Java)", group = "")
public class StoneDetection extends LinearOpMode {

    private VuforiaSkyStone vuforiaSkyStone;
    private TfodSkyStone tfodSkyStone;
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftRear = null;
    private DcMotor rightRear = null;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        List<Recognition> recognitions;
        int ImageHeight;
        float ObjectHeight;
        double ObjectAngle;
        double ObjectHeightRatio;
        double SkystoneCount;
        boolean SkystoneFound;
        double TargetHeightRatio;
        int adjustments = 0;
        leftFront = hardwareMap.get(DcMotor.class, "left_front");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");
        leftRear = hardwareMap.get(DcMotor.class, "left_rear");
        rightRear = hardwareMap.get(DcMotor.class, "right_rear");
        vuforiaSkyStone = new VuforiaSkyStone();
        tfodSkyStone = new TfodSkyStone();
        stopEncoders();
        telemetry.addData("Init ", "started");
        telemetry.update();
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        // Init Vuforia because Tensor Flow needs it.
        vuforiaSkyStone.initialize(
                "", // vuforiaLicenseKey
                VuforiaLocalizer.CameraDirection.BACK, // cameraDirection
                false, // useExtendedTracking
                true, // enableCameraMonitoring
                VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES, // cameraMonitorFeedback
                0, // dx
                0, // dy
                0, // dz
                0, // xAngle
                0, // yAngle
                0, // zAngle
                false); // useCompetitionFieldTargetLocations
        telemetry.addData("Vuforia", "initialized");
        telemetry.update();
        // Let's use 70% minimum confidence and
        // and no object tracker.
        tfodSkyStone.initialize(vuforiaSkyStone, 0.7F, false, true);
        telemetry.addData(">", "Press Play to start");
        telemetry.update();
        // Set target ratio of object height to image
        // height value corresponding to the length
        // of the robot's neck.
        TargetHeightRatio = 0.8;
        waitForStart();
        tfodSkyStone.activate();
        // We'll loop until gold block captured or time is up
        SkystoneFound = false;
        while (opModeIsActive() && !SkystoneFound) {
            // Get list of current recognitions.
            recognitions = tfodSkyStone.getRecognitions();
            // Report number of recognitions.
            telemetry.addData("Objects Recognized", recognitions.size());
            // If some objects detected...
            if (recognitions.size() > 0) {
                // ...let's count how many are gold.
                SkystoneCount = 0;
                // Step through the stones detected.
                for (Recognition recognition : recognitions) {
                    if (recognition.getLabel().equals("Skystone")) {
                        // A Skystone has been detected.
                        SkystoneCount = SkystoneCount + 1;
                        // We can assume this is the first Skystone
                        // because we break out of this loop below after
                        // using the information from the first Skystone.
                        // We don't need to calculate turn angle to Skystone
                        // because TensorFlow has estimated it for us.
                        ObjectAngle = recognition.estimateAngleToObject(AngleUnit.DEGREES);
                        // Negative angle means Skystone is left, else right.
                        telemetry.addData("Estimated Angle", ObjectAngle);
                        if (ObjectAngle > 0) {
                            telemetry.addData("Direction", "Right");
                            while (ObjectAngle < -25 || ObjectAngle > 25) {
                                rightRear.setPower(.5);
                                rightFront.setPower(-.5);
                                leftFront.setPower(.5);
                                leftRear.setPower(-.5);
                            }

                        } else {
                            telemetry.addData("Direction", "Left");

                            while (ObjectAngle < -25 || ObjectAngle > 25) {
                                rightRear.setPower(-.5);
                                rightFront.setPower(.5);
                                leftFront.setPower(-.5);
                                leftRear.setPower(.5);
                            }
                        }
                        ImageHeight = recognition.getImageHeight();
                        ObjectHeight = recognition.getHeight();
                        // Calculate height of Skystone relative to image height.
                        // Larger ratio means robot is closer to Skystone.
                        ObjectHeightRatio = ObjectHeight / ImageHeight;
                        telemetry.addData("HeightRatio", ObjectHeightRatio);
                        // Use height ratio to determine distance.
                        // If height ratio larger than (target - tolerance)...
                        if (ObjectHeightRatio < TargetHeightRatio - 0.08) {
                            // ...not close enough yet.
                            telemetry.addData("Distance", "Not close enough");
                            // If sum of turn powers are small
                            while (ObjectHeightRatio < TargetHeightRatio - 0.08) {
                                forward(.75);
                                ImageHeight = recognition.getImageHeight();
                                ObjectHeight = recognition.getHeight();
                                // Calculate height of Skystone relative to image height.
                                // Larger ratio means robot is closer to Skystone.
                                ObjectHeightRatio = ObjectHeight / ImageHeight;
                            }
                        }
                        // Else if height ratio more than (target+tolerance)...
                        else if (ObjectHeightRatio > TargetHeightRatio + 0.08) {
                            // ...robot too close to Skystone.
                            telemetry.addData("Distance", "Too close");
                            // If calculated turn power levels are small...
                            while (ObjectHeightRatio > TargetHeightRatio - 0.08) {
                                forward(-.75);
                                ImageHeight = recognition.getImageHeight();
                                ObjectHeight = recognition.getHeight();
                                // Calculate height of Skystone relative to image height.
                                // Larger ratio means robot is closer to Skystone.
                                ObjectHeightRatio = ObjectHeight / ImageHeight;
                            }
                        }
                        if (ObjectHeightRatio > (TargetHeightRatio - .08) && ObjectHeightRatio < TargetHeightRatio + .08) {
                            SkystoneFound = true;
                            telemetry.addData("done", "skystonedetected");
                        }

                    }
                }


                // If no Skystones detected...
                if (SkystoneCount == 0) {
                    telemetry.addData("Status", "No Skystone");
                    telemetry.addData("Action", "Back up");
                    // Back up slowly hoping to bring Skystone in view.
                    forward(-0.1);
                } else {
                    // No objects detected
                    telemetry.addData("Status", "No objects detected");
                    telemetry.addData("Action", "Back up");
                    // Back up slowly hoping to bring objects in view.
                    forward(-0.1);

                }
            }
            telemetry.update();

            // Skystone found, time is up or stop was requested.
            tfodSkyStone.deactivate();


            vuforiaSkyStone.close();
            tfodSkyStone.close();
        }
    }
        public void stopEncoders ()
        {
            leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        public void startEncoders () {
            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            ;
            rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        public void forward ( double power){
            leftFront.setPower(power);
            leftRear.setPower(power);
            rightFront.setPower(power);
            rightRear.setPower(power);
        }

        public void setPosition ( int pos){
            leftFront.setTargetPosition(pos);
            leftRear.setTargetPosition(pos);
            rightFront.setTargetPosition(pos);
            rightRear.setTargetPosition(pos);

            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while (leftRear.isBusy() && rightRear.isBusy() && leftFront.isBusy() && rightFront.isBusy()) {
            }
        }



}