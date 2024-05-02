#!/usr/bin/env python
import rospy
import rosbag
import numpy as np
from nav_msgs.msg import Odometry
import argparse
from tf.transformations import euler_from_quaternion


class OdomBagAnalyzer:
    def __init__(self, bag_file, topic):
        # Initialize lists to store x and y positions
        self.x_positions = []
        self.y_positions = []
        self.yaw_angles = []

        # Open the rosbag file
        self.bag_file = bag_file
        self.topic = topic
        print(f"Bag  : {self.bag_file}")
        print(f"Topic: {self.topic}")

    def analyze(self):
        self.bag = rosbag.Bag(self.bag_file)

        # Read messages from the odom topic
        for topic, msg, t in self.bag.read_messages(topics=[self.topic]):
            # Extract x and y positions
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y

            euler = euler_from_quaternion(
                [
                    msg.pose.pose.orientation.x,
                    msg.pose.pose.orientation.y,
                    msg.pose.pose.orientation.z,
                    msg.pose.pose.orientation.w,
                ]
            )

            # Append to the lists
            self.x_positions.append(x)
            self.y_positions.append(y)
            self.yaw_angles.append(euler[2])
            
        print(f"Total Time: {int(self.bag.get_end_time()-self.bag.get_start_time())}")
        
        # Close the rosbag file
        self.bag.close()
        # Calculate mean and standard deviation for x and y
        x_mean = np.mean(self.x_positions)
        y_mean = np.mean(self.y_positions)
        yaw_mean = np.mean(self.yaw_angles)
        x_std = np.std(self.x_positions)
        y_std = np.std(self.y_positions)
        yaw_std = np.std(self.yaw_angles)
        
        # Return the results
        return x_mean, y_mean, yaw_mean, x_std, y_std, yaw_std


if __name__ == "__main__":
    # Path to your rosbag file
    parser = argparse.ArgumentParser(
        description="Open a rosbag, and calculate the mean and stardard deviation of a odom topic"
    )
    parser.add_argument("-b", "--bagfile", type=str)
    parser.add_argument("-t", "--topic", type=str)

    args = parser.parse_args()

    # Create an instance of the OdomBagAnalyzer
    analyzer = OdomBagAnalyzer(args.bagfile, args.topic)
    
    # Perform the analysis
    x_mean, y_mean, yaw_mean, x_std, y_std, yaw_std = analyzer.analyze()
    
    # Print the results
    print(f"Mean X: {x_mean:.3}, Std Y: {x_std:.3}")
    print(f"Mean Y: {y_mean:.3}, Std Y: {y_std:.3}")
    print(f"Mean W: {yaw_mean:.3}, Std W: {yaw_std:.3}")
