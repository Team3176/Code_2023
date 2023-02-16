package team3176.robot.commands.arm.shoulder;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.networktables.PubSubOption;

public class DoNetworkTablesWork {
      // the publisher is an instance variable so its lifetime matches that of the class
  final DoublePublisher dblPub;

  public DoNetworkTablesWork(DoubleTopic dblTopic) {
    // start publishing; the return value must be retained (in this case, via
    // an instance variable)
    
    //dblPub = dblTopic.publish();

    // publish options may be specified using PubSubOption
    
    //dblPub = dblTopic.publish(PubSubOption.keepDuplicates(true));

    // publishEx provides additional options such as setting initial
    // properties and using a custom type string. Using a custom type string for
    // types other than raw and string is not recommended. The properties string
    // must be a JSON map.
    dblPub = dblTopic.publishEx("double", "{\"myprop\": 5}");
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
