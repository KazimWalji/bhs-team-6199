/* Copyright (c) 2017 FIRST. All rights reserved.
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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaSkyStone;
import org.firstinspires.ftc.robotcore.external.tfod.TfodSkyStone;


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

@Autonomous(name="Encoders Test", group="Linear Opmode")

public class encodersTest extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftRear = null;
    private DcMotor rightRear = null;
    private ElapsedTime runTime = new ElapsedTime();
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        leftFront = hardwareMap.get(DcMotor.class, "left_front");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");
        leftRear = hardwareMap.get(DcMotor.class, "left_rear");
        rightRear = hardwareMap.get(DcMotor.class, "right_rear");
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        stopEncoders();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode((DcMotor.RunMode.STOP_AND_RESET_ENCODER));

        leftFront.setTargetPosition(600);
        rightFront.setTargetPosition(-600);
        leftRear.setTargetPosition(600);
        rightRear.setTargetPosition(-600);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRear.setMode((DcMotor.RunMode.RUN_TO_POSITION));


        leftFront.setPower(.3);
        rightFront.setPower(.3);
        rightRear.setPower(.3);
        leftRear.setPower(.3);
        runTime.reset();
        while (leftFront.isBusy() && rightRear.isBusy() && leftRear.isBusy() && rightFront.isBusy() && opModeIsActive() && runTime.seconds() < 4) {
            telemetry.addData("Encoders", "Running");
            telemetry.update();
        }

        leftFront.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
        leftRear.setPower(0);



        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {


        }
    }

    public void stopEncoders()
    {
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void resetEncoders ()
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

        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void forward (double power){
        leftFront.setPower(power);
        leftRear.setPower(power);
        rightFront.setPower(power);
        rightRear.setPower(power);
    }

    public void setPosition ( int pos, double power){
        leftFront.setTargetPosition(pos);
        leftRear.setTargetPosition(pos);
        rightFront.setTargetPosition(pos);
        rightRear.setTargetPosition(pos);
        leftFront.setPower(power);
        leftRear.setPower(power);
        rightFront.setPower(power);
        rightRear.setPower(power);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (leftRear.isBusy() && rightRear.isBusy() && leftFront.isBusy() && rightFront.isBusy()) {
        }
    }



}
