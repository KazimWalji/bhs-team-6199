
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Arm Lift", group="Linear Opmode")
//@Disabled
public class arm_LIft extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftRear = null;
    private DcMotor rightRear = null;
    private Servo armServo = null;
    private DcMotor armMotor = null;

    @Override
    public void runOpMode() {
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftFront = hardwareMap.get(DcMotor.class, "left_front");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");
        leftRear  = hardwareMap.get(DcMotor.class, "left_rear");
        rightRear = hardwareMap.get(DcMotor.class, "right_rear");

        armMotor = hardwareMap.get(DcMotor.class, "arm_motor");
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode((DcMotor.RunMode.RUN_WITHOUT_ENCODER));
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));
        armServo = hardwareMap.get(Servo.class, "servo_arm");

        //Wait for the game to start (driver presses PLAY)
        waitForStart();



        //armServo.setPosition(0);
        runtime.reset();
        // Look for DPAD presses to change the selection



        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {


          if(gamepad1.left_stick_y<0)
          {
              armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
              armMotor.setTargetPosition(1000);
              armMotor.setPower(.3);
              armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
              while(armMotor.isBusy() && gamepad1.left_stick_y < 0)
              {

              }
              armMotor.setPower(0);
          }
            if(gamepad1.left_stick_y>0)
            {
                armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                armMotor.setTargetPosition(-1000);
                armMotor.setPower(.3);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while(armMotor.isBusy() && gamepad1.left_stick_y < 0)
                {

                }
                armMotor.setPower(0);
            }
            if (gamepad1.left_stick_y == 0)
            {
                armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

        }
    }
}