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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


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

@TeleOp(name="Tank", group="Linear Opmode")
public class tDrive extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor lDrive;
    private DcMotor rDrive;
    private DcMotor out;
    private DcMotor up;
    private DcMotor grab1;
    private DcMotor grab2;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //names from hardware map
        lDrive  = hardwareMap.get(DcMotor.class, "l");
        rDrive = hardwareMap.get(DcMotor.class, "r");
        out = hardwareMap.get(DcMotor.class, "o");
        up  = hardwareMap.get(DcMotor.class, "u1");
        grab1 = hardwareMap.get(DcMotor.class, "g1");
        grab2 = hardwareMap.get(DcMotor.class, "g2");

        /*how to reverse if needed
        lDrive.setDirection(DcMotor.Direction.FORWARD);
        rDrive.setDirection(DcMotor.Direction.REVERSE);
        */

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double leftPower;
            double rightPower;
            double outPower;
            double upPower;
            double grabPower;

            //classic tank drive
            leftPower = gamepad1.left_stick_y;
            rightPower = gamepad1.right_stick_y;


            //clip range
            leftPower = Range.clip(leftPower, -1, 1);
            rightPower = Range.clip(rightPower, -1, 1);

            //out move
            if(gamepad1.y){
                outPower = 1;
            }
            else if(gamepad1.a){
                outPower = -1;
            }
            else{
                outPower = 0;
            }


            //up and down
            if(gamepad2.y){
                upPower = 1;
            }
            else if(gamepad2.a){
                upPower = -1;
            }
            else{
                upPower = 0;
            }

            //grabber mechanism
            if(gamepad2.left_bumper){
                grabPower = 1;
            }
            else if(gamepad2.right_bumper){
                grabPower = -1;
            }
            else{
                grabPower = 0;
            }

            if(gamepad1.right_trigger > 0.1){
                leftPower = .5*leftPower;
                rightPower = .5*rightPower;
            }

            //set power for everything
            lDrive.setPower(leftPower);
            rDrive.setPower(rightPower);
            out.setPower(outPower);
            up.setPower(upPower);
            grab1.setPower(grabPower);
            grab2.setPower(-grabPower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f), up (%.2f), grab(%.2f)", leftPower, rightPower, upPower, grabPower);
            telemetry.update();
        }
    }


}
