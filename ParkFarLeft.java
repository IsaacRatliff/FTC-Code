package workspace;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Hardware;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Autonomous(name="5 Pt: Forward Left to Park", group="Linear Opmode")

public class ParkFarLeft extends AutoOp {

    @Override
    public void runOpMode(){
        initialize(hardwareMap, telemetry);
        waitForStart();
        forward(1);
        left(1);
        forward(1.5);
    }
}
