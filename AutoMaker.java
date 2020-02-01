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

package workspace;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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

@TeleOp(name="AutoMaker", group="Linear Opmode")

public class AutoMaker extends AutoOp{
    @Override
    public void runOpMode(){
        initialize(hardwareMap, telemetry);
        ElapsedTime timer = new ElapsedTime();
        String log = "init";
        String dir = "stop";
        String arm_dir = "stop";
        String finger_dir = "stop";
        int claw_pos = 2;
        double total_time = 0.0;
        double arm_static = 0.0;
        waitForStart();
        timer.reset();
        while(opModeIsActive()){
            if(gamepad1.dpad_up && !dir.equals("F")){
                leftWheels(0.3);
                rightWheels(0.3);
                log += String.format(" %.2f forward", timer.seconds() - total_time);
                total_time = timer.seconds();
                dir = "F";
            }
            else if(gamepad1.dpad_down && !dir.equals("B")){
                leftWheels(-0.3);
                rightWheels(-0.3);
                log += String.format(" %.2f backward", timer.seconds() - total_time);
                total_time = timer.seconds();
                dir = "B";
            }
            else if(gamepad1.dpad_left && !dir.equals("TL")){
                leftWheels(-0.3);
                rightWheels(0.3);
                log += String.format(" %.2f left", timer.seconds() - total_time);
                total_time = timer.seconds();
                dir = "TL";
            }
            else if(gamepad1.dpad_right && !dir.equals("TR")){
                leftWheels(0.3);
                rightWheels(-0.3);
                log += String.format(" %.2f right", timer.seconds() - total_time);
                total_time = timer.seconds();
                dir = "TR";
            }
            else if(!(gamepad1.dpad_up || gamepad1.dpad_down ||
                      gamepad1.dpad_left || gamepad1.dpad_right ||
                      dir.equals("stop"))){
                stopWheels();
                log += String.format(" %.2f stop", timer.seconds() - total_time);
                total_time = timer.seconds();
                dir = "stop";
            }
            
            if(gamepad1.left_bumper && !finger_dir.equals("up")){
                fingerUp();
                log += String.format(" %.2f finger up", timer.seconds() - total_time);
                total_time = timer.seconds();
                finger_dir = "up";
            }
            else if(gamepad1.left_trigger > 0.1 && !finger_dir.equals("down")){
                fingerDown();
                log += String.format(" %.2f finger down", timer.seconds() - total_time);
                total_time = timer.seconds();
                finger_dir = "down";
            }
            if (gamepad1.right_bumper && !arm_dir.equals("up")){
                arm.setPower(.3);
                log += String.format(" %.2f arm up", timer.seconds() - total_time);
                total_time = timer.seconds();
                arm_dir = "up";
            }
            else if(gamepad1.right_trigger > 0.1 && !arm_dir.equals("down")){
                arm.setPower(-0.3);
                log += String.format(" %.2f arm down", timer.seconds() - total_time);
                total_time = timer.seconds();
                arm_dir = "down";
            }
            else if(!(gamepad1.right_trigger > 0.1 || gamepad1.right_bumper ||
                      arm_dir.equals("stop"))){
                arm.setPower(arm_static);
                log += String.format(" %.2f arm stop", timer.seconds() - total_time);
                total_time = timer.seconds();
                arm_dir = "stop";
            }
            if(gamepad1.x && claw_pos != 1){  // close
                closeClaw();
                log += String.format(" %.2f claw close", timer.seconds() - total_time);
                total_time =  timer.seconds();
                claw_pos = 1;
            }
            else if(gamepad1.y && claw_pos != 0){  // open
                openClaw();
                log += String.format(" %.2f claw open", timer.seconds() - total_time);
                total_time =  timer.seconds();
                claw_pos = 0;
            }
            telemetry.addData("Log", log);
            telemetry.update();
        }
    }
}
