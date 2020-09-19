package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.Rect;

@Autonomous
public class RingDetectionCV extends LinearOpMode {
    OpenCvCamera phoneCam;
    StageSwitchingPipeline pipeline;

    //moves all rectangles right or left by amount. units are in ratio to monitor
    @Override
    public void runOpMode() {
        /*
         * Instantiate an OpenCvCamera object for the camera we'll be using.
         * In this sample, we're using the phone's internal camera. We pass it a
         * CameraDirection enum indicating whether to use the front or back facing
         * camera, as well as the view that we wish to use for camera monitor (on
         * the RC phone). If no camera monitor is desired, use the alternate
         * single-parameter constructor instead (commented out below)
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View
        //phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK);

        /*
         * Open the connection to the camera device
         */
        phoneCam.openCameraDevice();

        /*
         * Specify the image processing pipeline we wish to invoke upon receipt
         * of a frame from the camera. Note that switching pipelines on-the-fly
         * (while a streaming session is in flight) *IS* supported.
         */
        pipeline = new StageSwitchingPipeline();
        phoneCam.setPipeline(pipeline);

        /*
         * Tell the camera to start streaming images to us! Note that you must make sure
         * the resolution you specify is supported by the camera. If it is not, an exception
         * will be thrown.
         *
         * Also, we specify the rotation that the camera is used in. This is so that the image
         * from the camera sensor can be rotated such that it is always displayed with the image upright.
         * For a front facing camera, rotation is defined assuming the user is looking at the screen.
         * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
         * away from the user.
         */
        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

        /*
         * Wait for the user to press start on the Driver Station
         */


        waitForStart();

        while (opModeIsActive()) {
                telemetry.addData("pos: ", pipeline.getAnalysis());
                telemetry.addData("thresh: ", pipeline.getThreshold());
                telemetry.update();
            }
    }

    /*
     * An example image processing pipeline to be run upon receipt of each frame from the camera.
     * Note that the processFrame() method is called serially from the frame worker thread -
     * that is, a new camera frame will not come in while you're still processing a previous one.
     * In other words, the processFrame() method will never be called multiple times simultaneously.
     *
     * However, the rendering of your processed image to the viewport is done in parallel to the
     * frame worker thread. That is, the amount of time it takes to render the image to the
     * viewport does NOT impact the amount of frames per second that your pipeline can process.
     *
     * IMPORTANT NOTE: this pipeline is NOT invoked on your OpMode thread. It is invoked on the
     * frame worker thread. This should not be a problem in the vast majority of cases. However,
     * if you're doing something weird where you do need it synchronized with your OpMode thread,
     * then you will need to account for that accordingly.
     */
    public static class StageSwitchingPipeline extends OpenCvPipeline {
        enum ringPos
        {
            Four,
            One,
            None
        }
        static final Point topLeft = new Point(40,160);
        static final int width = 25;
        static final int height = 35;
        final int fourRingsThreshold = 100;
        final int oneRingThreshold = 122;
        Point pointA = new Point(topLeft.x, topLeft.y);
        Point pointB = new Point(topLeft.x + width, topLeft.y + height);

        Mat box = new Mat();
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avgCb;

        private volatile ringPos pos = ringPos.Four;

        void inputToCb(Mat input)
        {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 2);
        }

        @Override
        public void init(Mat firstFrame)
        {
            inputToCb(firstFrame);
            box = Cb.submat(new Rect(pointA, pointB));
        }
        @Override
        public Mat processFrame(Mat input) {
            inputToCb(input);
            avgCb = (int) Core.mean(box).val[0];
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    pointA, // First point which defines the rectangle
                    pointB, // Second point which defines the rectangle
                    new Scalar(0, 255, 0), 1);
            if(avgCb < oneRingThreshold)
            {
                pos = ringPos.One;
            }
            if(avgCb < fourRingsThreshold) {
                pos = ringPos.Four;
            }
            if(avgCb > 122)
            {
                pos = ringPos.None;
            }
            return input;
        }
        public ringPos getAnalysis()
        {
            return pos;
        }
        public int getThreshold()
        {
            return avgCb;
        }
    }
}