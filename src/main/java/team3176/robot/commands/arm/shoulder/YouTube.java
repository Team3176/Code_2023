package team3176.robot.commands.arm.shoulder;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.TimestampedDouble;

public class YouTube {

        // the subscriber is an instance variable so its lifetime matches that of the class
        final DoubleSubscriber dblSub;
      
        public YouTube(DoubleTopic dblTopic) {
          // start subscribing; the return value must be retained.
          // the parameter is the default value if no value is available when get() is called
          
          //dblSub = dblTopic.subscribe(0.0);
      
          // subscribe options may be specified using PubSubOption
          
          //dblSub = dblTopic.subscribe(0.0, PubSubOption.keepDuplicates(true), PubSubOption.pollStorage(10));
      
          // subscribeEx provides the options of using a custom type string.
          // Using a custom type string for types other than raw and string is not recommended.
          dblSub = dblTopic.subscribeEx("double", 0.0);
        }
      
        public void periodic() {
          // simple get of most recent value; if no value has been published,
          // returns the default value passed to the subscribe() function
          
          //double val = dblSub.get();
      
          // get the most recent value; if no value has been published, returns
          // the passed-in default value
          double val = dblSub.get(-1.0);
      
          // subscribers also implement the appropriate Supplier interface, e.g. DoubleSupplier
          
          //double val = dblSub.getAsDouble();
      
          // get the most recent value, along with its timestamp
          TimestampedDouble tsVal = dblSub.getAtomic();
      
          // read all value changes since the last call to readQueue/readQueueValues
          // readQueue() returns timestamps; readQueueValues() does not.
          TimestampedDouble[] tsUpdates = dblSub.readQueue();
          double[] valUpdates = dblSub.readQueueValues();
        }
      
        // often not required in robot code, unless this class doesn't exist for
        // the lifetime of the entire robot program, in which case close() needs to be
        // called to stop subscribing
        public void close() {
          // stop subscribing
          dblSub.close();
        }
      }

