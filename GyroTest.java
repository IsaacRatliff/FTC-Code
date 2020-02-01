package workspace;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Axis;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name="GyroTest")
public class GyroTest extends AutoOp {

    public void runOpMode() {
        initialize(hardwareMap, telemetry);
        //wheelPower = 0.2;
        waitForStart();
        left_gyro(1);
        pause(1);
        right_gyro(1);
        pause(1);
        left_gyro(2);
        pause(1);
        right_gyro(2);
        pause(1);
        left(8);
        pause(1);
        right(8);
    }
}