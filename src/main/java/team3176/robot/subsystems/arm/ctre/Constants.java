/**
 * Simple class containing constants used throughout project
 */
package team3176.robot.subsystems.arm.ctre;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.Topic;

public class Constants {

	public static final int kSlotIdx = 0;
	public static final int kPIDLoopIdx = 0;
	public static final int kTimeoutMs = 30;
	public static boolean kSensorPhase = true;
	public static boolean kMotorInvert = false;
    static final Gains kGains = new Gains(0.1, 0.0, 0.00, 0.0, 1, 0.50); 
	
	NetworkTableInstance inst = NetworkTableInstance.getDefault();
	NetworkTable table = inst.getTable("NetworkTables");

	// get a type-specific topic from a generic Topic
	Topic genericTopic = inst.getTopic("/NetworkTables/X");
	DoubleTopic dblTopic = new DoubleTopic(genericTopic);

	// the publisher is an instance variable so its lifetime matches that of the class
	final DoublePublisher dblPub;
  
	public Constants(DoubleTopic dblTopic) {
	  // start publishing; the return value must be retained (in this case, via
	  // an instance variable)
	  dblPub = dblTopic.publish();

	  dblPub.set(kPIDLoopIdx);
	}
  
	public void periodic() {
	  // publish a default value
	  dblPub.setDefault(0.0);
  
	  // publish a value with current timestamp
	  dblPub.set(1.0);
	  dblPub.set(2.0, 0);  // 0 = use current time
  
	  // publish a value with a specific timestamp; NetworkTablesJNI.now() can
	  // be used to get the current time. On the roboRIO, this is the same as
	  // the FPGA timestamp (e.g. RobotController.getFPGATime())
	  long time = NetworkTablesJNI.now();
	  dblPub.set(3.0, time);
  
	  // publishers also implement the appropriate Consumer functional interface;
	  // this example assumes void myFunc(DoubleConsumer func) exists
	  myFunc(dblPub);
	}
  
	private void myFunc(DoublePublisher dblPub2) {
	}

	// often not required in robot code, unless this class doesn't exist for
	// the lifetime of the entire robot program, in which case close() needs to be
	// called to stop publishing
	public void close() {
	  // stop publishing
	  dblPub.close();
	}
  }

  