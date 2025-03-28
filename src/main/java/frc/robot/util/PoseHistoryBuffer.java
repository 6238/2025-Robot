package frc.robot.util;

/*
 *******************************************************************************************
 * Copyright (C) 2017 FRC Team 1736 Robot Casserole - www.robotcasserole.org
 *******************************************************************************************
 *
 * This software is released under the MIT Licence - see the license.txt
 *  file in the root of this repo.
 *
 * Non-legally-binding statement from Team 1736:
 *  Thank you for taking the time to read through our software! We hope you
 *   find it educational and informative!
 *  Please feel free to snag our software for your own use in whatever project
 *   you have going on right now! We'd love to be able to help out! Shoot us
 *   any questions you may have, all our contact info should be on our website
 *   (listed above).
 *  If you happen to end up using our software to make money, that is wonderful!
 *   Robot Casserole is always looking for more sponsors, so we'd be very appreciative
 *   if you would consider donating to our club to help further STEM education.
 */

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.Timer;
import java.util.Arrays;

public class PoseHistoryBuffer {

  // I manually implemented a circular buffer cuz I enjoy difficult things like that.
  private int N; // size of buffers
  private Pose2d[] value_buffer; // circular buffer to hold all values
  private double[] time_buffer; // circular buffer to hold all values
  private int index; // "pointer" to the starting index in the buffers

  /**
   * This class implements a set of circular buffers that can be used to store values of signals in
   * the past, and retrieve values at arbitrary previous time indexes.
   *
   * <p>The value is assumed to saturate at the end of the maintained history.
   *
   * @param length Number of samples to keep
   * @param init_val Value to fill the inital samples with (probably zero is fine)
   */
  public PoseHistoryBuffer(int length, Pose2d init_val) {
    double init_time = Timer.getFPGATimestamp();
    value_buffer = new Pose2d[length];
    time_buffer = new double[length];
    Arrays.fill(value_buffer, init_val);
    Arrays.fill(time_buffer, init_time);
    index = 0;
    N = length;
  }

  /**
   * Insert a new value into the buffer. Discards the oldest value.
   *
   * @param time Time at which the value was sampled. elements must be inserted in a monotomically
   *     increasing fashion
   * @param value Value of the signal right now
   * @return true if the value was inserted, false otherwise
   */
  public boolean insert(double time, Pose2d value) {

    // Sanity check the inserted value
    if (time_buffer[index] > time) {
      System.out.println(
          "ERROR: InterpValueHistoryBuffer got non-increasing time vector. Tell software team!");
      return false;
    }

    // Update index
    index = getNextIdx(index);

    // Insert values
    value_buffer[index] = value;
    time_buffer[index] = time;

    return true;
  }

  /**
   * Returns the value at a given time, linearlly interpolated.
   *
   * @param time time at which to retrieve value
   * @return value at inputted time.
   */
  public Pose2d getValAtTime(double time) {
    int lower_idx = index;
    int upper_idx = getNextIdx(index);

    while (true) {
      if ((time_buffer[lower_idx] <= time) & (time_buffer[upper_idx] >= time)) {
        // Case, we've found the desired block. Calculate the value at the given time with linear
        // interpolation
        double segment_delta_t = time_buffer[upper_idx] - time_buffer[lower_idx];
        Transform2d segment_delta_v = value_buffer[upper_idx].minus(value_buffer[lower_idx]);
        double time_ratio = (time - time_buffer[lower_idx]) / segment_delta_t;
        return value_buffer[lower_idx].plus(segment_delta_v.times(time_ratio));
      }

      // Update iteration indicies for next loop
      lower_idx = getNextIdx(lower_idx);
      upper_idx = getNextIdx(upper_idx);

      if (upper_idx == index) {
        // Terminal case, we're off the end. Return the oldest value.
        return value_buffer[lower_idx];
      }
    }
  }

  private int getNextIdx(int idx) {
    if (idx + 1 >= N) {
      return 0;
    } else {
      return idx + 1;
    }
  }
}
