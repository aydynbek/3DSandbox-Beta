using System;
using Microsoft.Kinect;
using System.Diagnostics;
using System.Windows.Controls;
using System.Windows.Media.Media3D;

namespace _3DSandbox
{
    public enum GestureRunningState { Unknown, None, PanelRotation }

    public enum JointGeneralState { Unknown, Moving, Still }
    public enum ArmRotationalState { Unknown, RotatingDown, RotatingUp, RotatingClockWise, RotatingCntClockWise, NotRotating }
    public enum ArmPosition { Unknown, ArmStretchedOutHorizontally, ArmUp, ArmDown }
    public enum ArmRotationTrackingState { Unknown, Tracking, NotTracking, Initialized, Uninitialized }

    internal class Rotations
    {
        public Stopwatch stopwatch_rot;
        public Vector3D previous_frame_vector;
        public ArmRotationalState previous_rot_state = new ArmRotationalState();
        public int frame_counter;
        public ArmRotationTrackingState rot_tracking_state = new ArmRotationTrackingState();
        public double angle;
        public double[] previous_angles;
        public bool enough_time_at_angle;

        public const int SIGNAL_SAME_ANGLE_TIME = 1000;
        public const int PREV_ARRAY_LENGTH = 5;
        // Changed from one to three degrees..
        public const double angle_diff_sensitivity = 2;

        public Rotations()
        {
            stopwatch_rot = new Stopwatch();
            previous_angles = new double[PREV_ARRAY_LENGTH];
            previous_frame_vector = new Vector3D();
            previous_rot_state = ArmRotationalState.Unknown;
            frame_counter = 0;
            rot_tracking_state = ArmRotationTrackingState.Unknown;
            angle = 0;
            enough_time_at_angle = false;

            stopwatch_rot.Start();
        }

        public void managePrevAngles(double angle)
        {
            int i = PREV_ARRAY_LENGTH - 1;
            while (i > 0)
            {
                previous_angles[i] = previous_angles[i - 1];
                i--;
            }

            previous_angles[i] = angle;
        }

        public void stabializeAngle()
        {
            if (frame_counter > PREV_ARRAY_LENGTH)
            {
                if (Math.Abs(previous_angles[0] - previous_angles[PREV_ARRAY_LENGTH - 1]) > angle_diff_sensitivity)
                {
                    // Changing angle:
                    angle = previous_angles[0];

                    // Not the same angle, so no need to compare the times:
                    stopwatch_rot.Reset();
                    stopwatch_rot.Start();
                }
                else
                {
                    // Keeping the old angle:
                    previous_angles[PREV_ARRAY_LENGTH - 2] = angle;
                }

                if (stopwatch_rot.ElapsedMilliseconds > SIGNAL_SAME_ANGLE_TIME)
                {
                    // We have enough time at the same angle:
                    enough_time_at_angle = true;
                }
                else
                {
                    enough_time_at_angle = false;
                }
            }
            else
            {
                angle = previous_angles[0];
            }
        }
    }

    internal class Gesture_PanelRotation
    {
        internal const int BUFFER_LENGTH = 4;
        internal int buffer_filled_spots;

        internal double horizontal_stretch_sensitivity;

        internal const double angle_difference_sensitivity = .055;
        internal const double both_arms_down_sensitivity = .06;

        // Changed from .06...
        internal const double Z_distance_sensitivity = .10;

        internal const double trigg_2_short_limb_diff_sensitivity = .1;
        internal const double trigg_2_long_limb_diff_sensitivity = .15;
        internal const double trigg_2_Z_short_dist_sensitivity = .7;
        internal const double trigg_2_Z_long_dist_sensitivity = .15;
        internal const double trigg_2_horizontal_sensitivity = .05;

        internal double average_angle;

        internal Rotations left_arm_rotation;
        internal Rotations right_arm_rotation;

        ArmPosition left_arm_position;
        ArmPosition right_arm_position;

        internal Vector3D shoulder_to_elbow_left;
        internal Vector3D shoulder_to_elbow_right;

        internal double[] angle_buffer;

        public Gesture_PanelRotation(ArmPosition left_arm_position, ArmPosition right_arm_position,
                                    double horizontal_stretch_sensitivity)
        {
            angle_buffer = new double[BUFFER_LENGTH];
            buffer_filled_spots = 0;

            this.horizontal_stretch_sensitivity = horizontal_stretch_sensitivity;

            left_arm_rotation = new Rotations();
            right_arm_rotation = new Rotations();

            this.left_arm_position = left_arm_position;
            this.right_arm_position = right_arm_position;

            shoulder_to_elbow_left = new Vector3D();
            shoulder_to_elbow_right = new Vector3D();
            average_angle = 0.0;
        }

        internal static bool checkPanelRotationGesture(double horizontal_stretch_sensitivity,
                                    ref ArmPosition left_arm_position, ref ArmPosition right_arm_position)
        {
            // Calling the static method to update the arm position states:
            return panelRotationInitialCondition(horizontal_stretch_sensitivity,
                                       ref left_arm_position, ref right_arm_position);
        }

        internal void initializeGesture()
        {
            // Initiation successful, we can signal the beginning of rotation angle reading:
            left_arm_rotation.rot_tracking_state = ArmRotationTrackingState.Initialized;
            right_arm_rotation.rot_tracking_state = ArmRotationTrackingState.Initialized;

            // Initialing the rotation state:
            left_arm_rotation.previous_rot_state = ArmRotationalState.NotRotating;
            right_arm_rotation.previous_rot_state = ArmRotationalState.NotRotating;

            // Passing this shoulder to elbow vector so that on the next event we can compare:
            left_arm_rotation.previous_frame_vector = GestureAuxilaryMethods.updateLimbVectors(JointType.ShoulderLeft,
                                                                        JointType.ElbowLeft);
            right_arm_rotation.previous_frame_vector = GestureAuxilaryMethods.updateLimbVectors(JointType.ShoulderRight,
                                                                        JointType.ElbowRight);

        }

        // This method returns 0 if everything went well, -1 if reading of the gesture is stopped:
        internal int readArmRotation(JointGeneralState left_elbow_general_state, JointGeneralState right_elbow_general_state)
        {
            // Moved these two lines out of the if statement:
            shoulder_to_elbow_left = GestureAuxilaryMethods.updateLimbVectors(JointType.ShoulderLeft, JointType.ElbowLeft);
            shoulder_to_elbow_right = GestureAuxilaryMethods.updateLimbVectors(JointType.ShoulderRight, JointType.ElbowRight);

            // Checking the boundary conditions to see if we can actually do the reading:
            if (boundaryConditions(left_elbow_general_state, right_elbow_general_state) == true &&
               (left_arm_rotation.rot_tracking_state == ArmRotationTrackingState.Tracking ||
               left_arm_rotation.rot_tracking_state == ArmRotationTrackingState.Initialized) &&
               (right_arm_rotation.rot_tracking_state == ArmRotationTrackingState.Tracking ||
               right_arm_rotation.rot_tracking_state == ArmRotationTrackingState.Initialized))
            {
                left_arm_rotation.rot_tracking_state = ArmRotationTrackingState.Tracking;
                right_arm_rotation.rot_tracking_state = ArmRotationTrackingState.Tracking;
            }
            else
            {
                left_arm_rotation.rot_tracking_state = ArmRotationTrackingState.NotTracking;
                right_arm_rotation.rot_tracking_state = ArmRotationTrackingState.NotTracking;
            }

            if (left_arm_rotation.rot_tracking_state == ArmRotationTrackingState.Tracking &&
                right_arm_rotation.rot_tracking_state == ArmRotationTrackingState.Tracking)
            {
                updateRotationStatus();
                updateArmStatus();

                //TextInformation.insert_main_text_block("Left Arm: " + left_arm_rotation.previous_angles[0].ToString(), 2);
                //TextInformation.insert_main_text_block("Right Arm: " + right_arm_rotation.previous_angles[0].ToString(), 2);

                // Lets stabialize the angle:
                left_arm_rotation.stabializeAngle();
                right_arm_rotation.stabializeAngle();

                TextInformation.insert_main_text_block("Angle Left Arm: " + left_arm_rotation.angle.ToString("n4"), 1);
                TextInformation.insert_main_text_block("Angle Right Arm: " + right_arm_rotation.angle.ToString("n4"), 1);

                if (left_arm_rotation.enough_time_at_angle == true && right_arm_rotation.enough_time_at_angle == true)
                {
                    TextInformation.insert_main_text_block("Sending Signal", 1);

                    // Terminating the gesture reading. This might be changed later on:
                    //left_arm_rotation.rot_tracking_state = ArmRotationTrackingState.NotTracking;
                    //right_arm_rotation.rot_tracking_state = ArmRotationTrackingState.NotTracking;
                }
                else
                {
                    TextInformation.insert_main_text_block("NOT Sending Signal", 1);
                }

                left_arm_rotation.frame_counter++;
                right_arm_rotation.frame_counter++;

                // Updating previous vectors:
                left_arm_rotation.previous_frame_vector = shoulder_to_elbow_left;
                right_arm_rotation.previous_frame_vector = shoulder_to_elbow_right;

                return 0;
            }
            else
            {
                left_arm_rotation.previous_rot_state = ArmRotationalState.NotRotating;
                right_arm_rotation.previous_rot_state = ArmRotationalState.NotRotating;
                return -1;
            }
        }

        // This method calculates the current angle and inserts it into the array to be stabialized:
        internal void updateArmStatus()
        {
            double left_Y_difference_signed = GesturesMasterControl.body.Joints[JointType.WristLeft].Position.Y -
                               GesturesMasterControl.body.Joints[JointType.SpineShoulder].Position.Y;
            double right_Y_difference_signed = GesturesMasterControl.body.Joints[JointType.WristRight].Position.Y -
                                GesturesMasterControl.body.Joints[JointType.SpineShoulder].Position.Y;
            double left_X_difference_signed = GesturesMasterControl.body.Joints[JointType.WristLeft].Position.X -
                               GesturesMasterControl.body.Joints[JointType.SpineShoulder].Position.X;
            double right_X_difference_signed = GesturesMasterControl.body.Joints[JointType.WristRight].Position.X -
                                GesturesMasterControl.body.Joints[JointType.SpineShoulder].Position.X;

            //left_arm_rotation.angle = 180 + (180 * (Math.Atan(left_Y_difference_signed/left_X_difference_signed) / Math.PI));
            //right_arm_rotation.angle = 180 * (Math.Atan(right_Y_difference_signed / right_X_difference_signed) / Math.PI);

            left_arm_rotation.managePrevAngles(180 + (180 * (Math.Atan(left_Y_difference_signed / left_X_difference_signed) / Math.PI)));
            right_arm_rotation.managePrevAngles(180 * (Math.Atan(right_Y_difference_signed / right_X_difference_signed) / Math.PI));

            //average_angle = (Math.Abs(180 * (Math.Atan(left_Y_difference_signed / left_X_difference_signed) / Math.PI))
            //    + Math.Abs(180 * (Math.Atan(right_Y_difference_signed / right_X_difference_signed) / Math.PI))) / 2;

            left_Y_difference_signed = Math.Abs(left_Y_difference_signed);
            right_Y_difference_signed = Math.Abs(right_Y_difference_signed);

        }

        internal void updateRotationStatus()
        {
            // We check to see the difference between the Y positions of the elbow of previous and current vectors: 
            double left_Y_difference = left_arm_rotation.previous_frame_vector.Y - shoulder_to_elbow_left.Y;
            double right_Y_difference = right_arm_rotation.previous_frame_vector.Y - shoulder_to_elbow_right.Y;

            if (left_Y_difference <= 0 && right_Y_difference >= 0)
            {
                left_arm_rotation.previous_rot_state = ArmRotationalState.RotatingClockWise;
                right_arm_rotation.previous_rot_state = ArmRotationalState.RotatingClockWise;
            }
            else if (left_Y_difference >= 0 && right_Y_difference < 0)
            {
                left_arm_rotation.previous_rot_state = ArmRotationalState.RotatingCntClockWise;
                right_arm_rotation.previous_rot_state = ArmRotationalState.RotatingCntClockWise;
            }
            else
            {
                left_arm_rotation.previous_rot_state = ArmRotationalState.NotRotating;
                right_arm_rotation.previous_rot_state = ArmRotationalState.NotRotating;
            }

            TextInformation.insert_main_text_block("Rotation: " + right_arm_rotation.previous_rot_state.ToString(), 1);
        }

        internal bool boundaryConditions(JointGeneralState left_elbow_general_state, JointGeneralState right_elbow_general_state)
        {
            bool angle_shoulder_elbow_steady = false, both_arms_same_dir = false, speed_normal = false,
            arms_Z_pos_constant = false, both_elbows_same_state = false;
            double left_Y_difference = 0, right_Y_difference = 0;

            // PREVIOUSLY: went from shoulder to elbow:
            // We check to see if the angle between spine shoulder and wrist for both arms are close: 
            double left_Y_difference_signed = GesturesMasterControl.body.Joints[JointType.WristLeft].Position.Y -
                                GesturesMasterControl.body.Joints[JointType.SpineShoulder].Position.Y;
            double right_Y_difference_signed = GesturesMasterControl.body.Joints[JointType.WristLeft].Position.Y -
                                GesturesMasterControl.body.Joints[JointType.SpineShoulder].Position.Y;

            double left_Y_difference_signed_elbow = GesturesMasterControl.body.Joints[JointType.ElbowLeft].Position.Y -
                                GesturesMasterControl.body.Joints[JointType.ShoulderLeft].Position.Y;
            double right_Y_difference_signed_elbow = GesturesMasterControl.body.Joints[JointType.ElbowRight].Position.Y -
                                GesturesMasterControl.body.Joints[JointType.ShoulderRight].Position.Y;

            left_Y_difference = Math.Abs(left_Y_difference_signed);
            right_Y_difference = Math.Abs(right_Y_difference_signed);

            //TextInformation.insert_main_text_block("Left Y Difference: " + left_Y_difference.ToString("n4"), 1);
            //TextInformation.insert_main_text_block("Right Y Difference: " + right_Y_difference.ToString("n4"), 1);
            //TextInformation.insert_main_text_block("Difference Between: " + (left_Y_difference - right_Y_difference).ToString("n4"), 1);

            if (Math.Abs(left_Y_difference - right_Y_difference) > angle_difference_sensitivity)
            {
                angle_shoulder_elbow_steady = false;
            }
            else
            {
                angle_shoulder_elbow_steady = true;
            }

            // Reusing the same variables for the elbow shoulder difference measurement:
            left_Y_difference = Math.Abs(left_Y_difference_signed_elbow);
            right_Y_difference = Math.Abs(right_Y_difference_signed_elbow);

            // If the arms are not stretched out horizontally, we can check to see if both arms are down or up:
            if (left_Y_difference > both_arms_down_sensitivity && right_Y_difference > both_arms_down_sensitivity)
            {
                // Check to see if both variables have the same sign:
                if (left_Y_difference_signed_elbow > 0 && right_Y_difference_signed_elbow > 0)
                {
                    TextInformation.insert_main_text_block("IIIIIII", 4);
                    both_arms_same_dir = true;
                }
                else if (left_Y_difference_signed_elbow < 0 && right_Y_difference_signed_elbow < 0)
                {
                    TextInformation.insert_main_text_block("OOOOOOO", 4);
                    both_arms_same_dir = true;
                }
                else
                {
                    both_arms_same_dir = false;
                }
            }

            // We need to also check to see if the Z distances between shoulder and elbow are close enough:
            double left_Z_difference = GesturesMasterControl.body.Joints[JointType.ElbowLeft].Position.Z -
                                GesturesMasterControl.body.Joints[JointType.ShoulderLeft].Position.Z;
            double right_Z_difference = GesturesMasterControl.body.Joints[JointType.ElbowRight].Position.Z -
                                GesturesMasterControl.body.Joints[JointType.ShoulderRight].Position.Z;

            //TextInformation.insert_main_text_block("Left Z Difference: " + left_Z_difference.ToString("n4"), 2);
            //TextInformation.insert_main_text_block("Right Z Difference: " + right_Z_difference.ToString("n4"), 2);

            left_Z_difference = Math.Abs(left_Z_difference);
            right_Z_difference = Math.Abs(right_Z_difference);

            if (left_Z_difference > Z_distance_sensitivity || right_Z_difference > Z_distance_sensitivity)
            {
                arms_Z_pos_constant = false;
            }
            else
            {
                arms_Z_pos_constant = true;
            }

            // Lets check if one elbow is moving and other is still: 
            // Very hard to make useful... not implemented
            if (left_elbow_general_state == JointGeneralState.Moving && right_elbow_general_state == JointGeneralState.Still)
            {
                both_elbows_same_state = false;
            }
            else if (left_elbow_general_state == JointGeneralState.Still && right_elbow_general_state == JointGeneralState.Moving)
            {
                both_elbows_same_state = false;
            }
            else if (left_elbow_general_state == JointGeneralState.Still && right_elbow_general_state == JointGeneralState.Still)
            {
                both_elbows_same_state = true;
            }
            else if (left_elbow_general_state == JointGeneralState.Moving && right_elbow_general_state == JointGeneralState.Moving)
            {
                both_elbows_same_state = true;
            }

            // Lets check the speed of rotation from one frame to another:
            // Not used due to poor functionality:
            double Y_single_frame_diff_left = shoulder_to_elbow_left.Y - left_arm_rotation.previous_frame_vector.Y;
            double Y_single_frame_diff_right = shoulder_to_elbow_right.Y - right_arm_rotation.previous_frame_vector.Y;

            TextInformation.insert_main_text_block("Left Speed: " + Y_single_frame_diff_left.ToString("n4"), 1);
            TextInformation.insert_main_text_block("Right Speed: " + Y_single_frame_diff_right.ToString("n4"), 1);

            if (Math.Abs(Y_single_frame_diff_left) < .015 && Math.Abs(Y_single_frame_diff_right) < .015)
            {
                speed_normal = true;
            }

            TextInformation.insert_main_text_block("Arms Z Position Constant: " + arms_Z_pos_constant.ToString(), 2);
            TextInformation.insert_main_text_block("Both Arms Same Direction: " + both_arms_same_dir.ToString(), 2);
            TextInformation.insert_main_text_block("Angle Shoulder Elbow Steady: " + angle_shoulder_elbow_steady.ToString(), 2);
            // Combining all booleans:
            if (angle_shoulder_elbow_steady == true && both_arms_same_dir == false && arms_Z_pos_constant == true)
            {
                TextInformation.insert_main_text_block("Boundary Clear", 2);
                return true;
            }
            else
            {
                TextInformation.insert_main_text_block("Boundary NOT Clear", 2);
                return false;
            }
        }

        internal static bool panelRotationInitialCondition(double horizontal_stretch_sensitivity,
                                        ref ArmPosition left_arm_position, ref ArmPosition right_arm_position)
        {
            bool ret_bool;

            /*ret_bool = Gesture_PanelRotation.triggerPosition_1(horizontal_stretch_sensitivity, ref left_arm_position,
                                                                   ref right_arm_position);*/

            ret_bool = triggerPosition_2(.3, .3, ref left_arm_position,
                                                                    ref right_arm_position);
            return ret_bool;
        }

        /// <summary>
        /// This method is designed to trigger a gesture at any angle, not just horizontally stretched.
        /// </summary>
        /// <param name="vert_arm_diff_sensitivity"></param>
        /// <param name="position_sensitivity"></param>
        /// <param name="left_arm_position"></param>
        /// <param name="right_arm_position"></param>
        /// <returns></returns>
        private static bool triggerPosition_2(double vert_arm_diff_sensitivity, double position_sensitivity,
                                       ref ArmPosition left_arm_position, ref ArmPosition right_arm_position)
        {
            bool short_arm_limb_steady = false, long_arm_limb_steady = false, right_Z_dist_steady = false,
                left_Z_dist_steady = false;

            // We get the Y differences between all arm joints:
            double left_Y_shoul_elbow_diff = GesturesMasterControl.body.Joints[JointType.ShoulderLeft].Position.Y -
                                GesturesMasterControl.body.Joints[JointType.ElbowLeft].Position.Y;
            double right_Y_shoul_elbow_diff = GesturesMasterControl.body.Joints[JointType.ShoulderRight].Position.Y -
                                 GesturesMasterControl.body.Joints[JointType.ElbowRight].Position.Y;

            double left_Y_elbow_wrist_diff = GesturesMasterControl.body.Joints[JointType.ElbowLeft].Position.Y -
                                GesturesMasterControl.body.Joints[JointType.WristLeft].Position.Y;
            double right_Y_elbow_wrist_diff = GesturesMasterControl.body.Joints[JointType.ElbowRight].Position.Y -
                                 GesturesMasterControl.body.Joints[JointType.WristRight].Position.Y;

            double left_Y_shoul_wrist_diff = GesturesMasterControl.body.Joints[JointType.ShoulderLeft].Position.Y -
                                GesturesMasterControl.body.Joints[JointType.WristLeft].Position.Y;
            double right_Y_shoul_wrist_diff = GesturesMasterControl.body.Joints[JointType.ShoulderRight].Position.Y -
                                 GesturesMasterControl.body.Joints[JointType.WristRight].Position.Y;

            double shoulder_spine_Y = GesturesMasterControl.body.Joints[JointType.SpineShoulder].Position.Y;
            double left_elbow_Y = GesturesMasterControl.body.Joints[JointType.ElbowLeft].Position.Y;
            double right_elbow_Y = GesturesMasterControl.body.Joints[JointType.ElbowRight].Position.Y;

            // First lets check to see if the elbows are not both above or below the spine shoulder:
            if ((left_elbow_Y - shoulder_spine_Y > 0 && right_elbow_Y - shoulder_spine_Y > 0) ||
                (left_elbow_Y - shoulder_spine_Y < 0 && right_elbow_Y - shoulder_spine_Y < 0))
            {
                // Check if both elbows are not horizontal:
                if (Math.Abs(left_elbow_Y - shoulder_spine_Y) > trigg_2_horizontal_sensitivity &&
                    Math.Abs(right_elbow_Y - shoulder_spine_Y) > trigg_2_horizontal_sensitivity)
                {
                    return false;
                }
            }

            left_Y_shoul_elbow_diff = Math.Abs(left_Y_shoul_elbow_diff);
            right_Y_shoul_elbow_diff = Math.Abs(right_Y_shoul_elbow_diff);
            left_Y_elbow_wrist_diff = Math.Abs(left_Y_elbow_wrist_diff);
            right_Y_elbow_wrist_diff = Math.Abs(right_Y_elbow_wrist_diff);
            left_Y_shoul_wrist_diff = Math.Abs(left_Y_shoul_wrist_diff);
            right_Y_shoul_wrist_diff = Math.Abs(right_Y_shoul_wrist_diff);

            //TextInformation.insert_main_text_block("Y Shoulder Elbow Diff: " + Math.Abs(left_Y_shoul_elbow_diff - right_Y_shoul_elbow_diff).ToString("n5"), 4);
            //TextInformation.insert_main_text_block("Y Elbow Wrist Diff: " + Math.Abs(left_Y_shoul_elbow_diff - right_Y_shoul_elbow_diff).ToString("n5"), 4);

            // We need to see if the differences between the first two pairs (short limbs) are within a limit:
            if ((Math.Abs(left_Y_shoul_elbow_diff - right_Y_shoul_elbow_diff) < trigg_2_short_limb_diff_sensitivity) &&
                (Math.Abs(left_Y_elbow_wrist_diff - right_Y_elbow_wrist_diff) < trigg_2_short_limb_diff_sensitivity))
            {
                short_arm_limb_steady = true;
            }

            //TextInformation.insert_main_text_block("Y Long Diff: " + Math.Abs(left_Y_shoul_wrist_diff - right_Y_shoul_wrist_diff).ToString("n5"), 4);

            // We need to see if the differences between the last pair (short limb) is within a limit:
            if ((Math.Abs(left_Y_shoul_wrist_diff - right_Y_shoul_wrist_diff) < trigg_2_long_limb_diff_sensitivity))
            {
                long_arm_limb_steady = true;
            }

            // We check the Z distances from the camera:
            double left_shoulder_Z = GesturesMasterControl.body.Joints[JointType.ShoulderLeft].Position.Z;
            double right_shoulder_Z = GesturesMasterControl.body.Joints[JointType.ShoulderRight].Position.Z;
            double left_elbow_Z = GesturesMasterControl.body.Joints[JointType.ElbowLeft].Position.Z;
            double right_elbow_Z = GesturesMasterControl.body.Joints[JointType.ElbowRight].Position.Z;
            double left_wrist_Z = GesturesMasterControl.body.Joints[JointType.WristLeft].Position.Z;
            double right_wrist_Z = GesturesMasterControl.body.Joints[JointType.WristRight].Position.Z;

            /*TextInformation.insert_main_text_block("Left Shoulder Z: " + left_shoulder_Z.ToString("n5"), 4);
            TextInformation.insert_main_text_block("Right Shoulder Z: " + right_shoulder_Z.ToString("n5"), 4);
            TextInformation.insert_main_text_block("Left Elbow Z: " + left_elbow_Z.ToString("n5"), 4);
            TextInformation.insert_main_text_block("Right Shoulder Z: " + right_elbow_Z.ToString("n5"), 4);
            TextInformation.insert_main_text_block("Left Wrist Z: " + left_wrist_Z.ToString("n5"), 4);
            TextInformation.insert_main_text_block("Right Wrist Z: " + right_wrist_Z.ToString("n5"), 4);
            */
            if ((Math.Abs(right_shoulder_Z - right_elbow_Z) < trigg_2_Z_short_dist_sensitivity) &&
                (Math.Abs(right_elbow_Z - right_wrist_Z) < trigg_2_Z_short_dist_sensitivity) &&
                (Math.Abs(right_shoulder_Z - right_wrist_Z) < trigg_2_Z_long_dist_sensitivity))
            {
                right_Z_dist_steady = true;
            }

            if ((Math.Abs(left_shoulder_Z - left_elbow_Z) < trigg_2_Z_short_dist_sensitivity) &&
                (Math.Abs(left_elbow_Z - left_wrist_Z) < trigg_2_Z_short_dist_sensitivity) &&
                (Math.Abs(left_shoulder_Z - left_wrist_Z) < trigg_2_Z_long_dist_sensitivity))
            {
                left_Z_dist_steady = true;
            }

            if (short_arm_limb_steady == true && long_arm_limb_steady == true &&
                right_Z_dist_steady == true && left_Z_dist_steady == true)
            {
                return true;
            }
            else
            {
                return false;
            }

        }

        // This trigger position is when the arms are stretched out horizontally: 
        private static bool triggerPosition_1(double horizontal_stretch_sensitivity,
                                        ref ArmPosition left_arm_position, ref ArmPosition right_arm_position)
        {
            bool ret_bool = false;

            // We first check to see if ShoulderCenter is above the two other shoulders:
            if (GesturesMasterControl.body.Joints[JointType.SpineShoulder].Position.Y >
                GesturesMasterControl.body.Joints[JointType.ShoulderLeft].Position.Y &&
               GesturesMasterControl.body.Joints[JointType.SpineShoulder].Position.Y >
               GesturesMasterControl.body.Joints[JointType.ShoulderRight].Position.Y)
            {
                double shoulder_left_Y = GesturesMasterControl.body.Joints[JointType.ShoulderLeft].Position.Y;
                double shoulder_right_Y = GesturesMasterControl.body.Joints[JointType.ShoulderRight].Position.Y;
                double elbow_left_Y = GesturesMasterControl.body.Joints[JointType.ElbowLeft].Position.Y;
                double elbow_right_Y = GesturesMasterControl.body.Joints[JointType.ElbowRight].Position.Y;

                // Then we check to see if the height between shoulder and elbow is not too much:
                if (Math.Abs(shoulder_left_Y - elbow_left_Y) < horizontal_stretch_sensitivity &&
                   Math.Abs(shoulder_right_Y - elbow_right_Y) < horizontal_stretch_sensitivity)
                {
                    left_arm_position = ArmPosition.ArmStretchedOutHorizontally;
                    right_arm_position = ArmPosition.ArmStretchedOutHorizontally;
                    ret_bool = true;
                }
                else
                {
                    left_arm_position = ArmPosition.Unknown;
                    right_arm_position = ArmPosition.Unknown;
                }
            }
            return ret_bool;
        }
    }

    public class GesturesMasterControl
    {
        // Static member:
        public static Body body;

        public Stopwatch stopWatch;
        public Stopwatch stopWatch_single_event;
        public Stopwatch stopWatch_trigger;

        public GestureRunningState running_gesture = new GestureRunningState();

        Gesture_PanelRotation panel_rotation;

        public Vector3D shoulder_to_elbow_left;
        public Vector3D shoulder_to_elbow_right;
        public Vector3D elbow_to_hand_left;
        public Vector3D elbow_to_hand_right;

        JointGeneralState right_elbow_general_state = new JointGeneralState();
        JointGeneralState left_elbow_general_state = new JointGeneralState();
        ArmPosition left_arm_position = new ArmPosition();
        ArmPosition right_arm_position = new ArmPosition();

        public Joint[] lf_elbow_prev_pos;
        public Joint[] rt_elbow_prev_pos;

        /* Previous values: 15 and .2*/
        //public const int PREV_FRAMES_ARRAY_LENGTH = 15;
        //public const double stillness_sensitivity = .2;

        /* New methodology: instead of cheching the standard deviation, we will now compare the position of 
           the first array element with the last array element: */
        public const int PREV_FRAMES_ARRAY_LENGTH = 5;
        public const double stillness_sensitivity = .005;
        public const double horizontal_stretch_sensitivity = .08;

        /* 1. Select a person most likely for gesture
            2. Wait until triggering conditions are met
            3. Wait for 0.5 seconds when triggered before starting the reading
            4. Read while checking the boundaries
            5. When a rotation is made, and the hands are still again, send a bluetooth signal
                a. at each correct angle reading, measure the time spent at that angle
                b. if the time exceeds some threshhold (1 second), send a signal
                c. if the person wants to continue rotating, he needs to wait a particular amount of time (based on motor rotation speed)
                d. reading to commence again for a new rotation
            6. Wait until another rotation is made */

        /* Constructor: */
        public GesturesMasterControl()
        {
            stopWatch = new Stopwatch();
            stopWatch_single_event = new Stopwatch();
            stopWatch_trigger = new Stopwatch();

            running_gesture = GestureRunningState.Unknown;

            right_elbow_general_state = JointGeneralState.Unknown;
            left_elbow_general_state = JointGeneralState.Unknown;
            right_arm_position = ArmPosition.Unknown;
            left_arm_position = ArmPosition.Unknown;

            lf_elbow_prev_pos = new Joint[PREV_FRAMES_ARRAY_LENGTH];
            rt_elbow_prev_pos = new Joint[PREV_FRAMES_ARRAY_LENGTH];

            shoulder_to_elbow_left = new Vector3D();
            shoulder_to_elbow_right = new Vector3D();
            elbow_to_hand_left = new Vector3D();
            elbow_to_hand_right = new Vector3D();
        }

        /* Methods: */
        public void setBody(Body _body)
        {
            body = _body;
        }

        public void runGestureAnalysis()
        {
            stopWatch_single_event.Reset();
            stopWatch_single_event.Start();
            TextInformation.insert_main_text_block("Stopwatch: " + stopWatch.Elapsed, 3);
            TextInformation.insert_main_text_block("Running Gesture: " + running_gesture.ToString(), 1);

            // I have moved the next 4 blocks of codes out of the if statement...
            /* Searching State: CHANGED FROM WRIST TO ELBOW */
            Joint left_elbow = body.Joints[JointType.ElbowLeft];
            Joint right_elbow = body.Joints[JointType.ElbowRight];

            // We make a call to this method to store previous positions:
            GestureAuxilaryMethods.managePrevArray(left_elbow, ref lf_elbow_prev_pos, PREV_FRAMES_ARRAY_LENGTH);
            GestureAuxilaryMethods.managePrevArray(right_elbow, ref rt_elbow_prev_pos, PREV_FRAMES_ARRAY_LENGTH);

            // We make a call to the following methods in order to update the state of elbows:
            GestureAuxilaryMethods.isJointStable(stillness_sensitivity, JointType.ElbowLeft, lf_elbow_prev_pos, PREV_FRAMES_ARRAY_LENGTH,
                                                    ref left_elbow_general_state);
            GestureAuxilaryMethods.isJointStable(stillness_sensitivity, JointType.ElbowRight, rt_elbow_prev_pos, PREV_FRAMES_ARRAY_LENGTH,
                                                    ref right_elbow_general_state);

            // The if else block below is to see if we are in the searching (for gesture) state or reading (the gesture) state:
            if (running_gesture == GestureRunningState.None || running_gesture == GestureRunningState.Unknown)
            {
                if (left_elbow_general_state == JointGeneralState.Still && right_elbow_general_state == JointGeneralState.Still)
                {
                    // Calling this method to see if a panel rotation gesture is possible:
                    if (Gesture_PanelRotation.checkPanelRotationGesture(horizontal_stretch_sensitivity, ref left_arm_position, ref right_arm_position))
                    {
                        // We can now initialize a PanelRotation class and go on further:
                        panel_rotation = new Gesture_PanelRotation(left_arm_position, right_arm_position, horizontal_stretch_sensitivity);
                        panel_rotation.initializeGesture();

                        // Update the gesture state:
                        running_gesture = GestureRunningState.PanelRotation;

                        // Start the stopwatch, we will wait .5 seconds for the gesture reading to begin:
                        stopWatch_trigger.Start();
                    }
                }
            }
            else      /* Reading State: */
            {
                if (stopWatch_trigger.ElapsedMilliseconds > 500)
                {
                    int reading_result = 0;

                    // There is a gesture that is to be read, we need to check which:
                    if (running_gesture == GestureRunningState.PanelRotation)
                    {
                        reading_result = panel_rotation.readArmRotation(left_elbow_general_state, right_elbow_general_state);
                    }

                    if (reading_result == -1)
                    {
                        // Discontinue the reading of the gesture:
                        running_gesture = GestureRunningState.None;

                        // Clear out the previous hand positions arrays:
                        lf_elbow_prev_pos = new Joint[PREV_FRAMES_ARRAY_LENGTH];
                        rt_elbow_prev_pos = new Joint[PREV_FRAMES_ARRAY_LENGTH];

                        // Clear out the statuses of Limbs: 
                        left_elbow_general_state = JointGeneralState.Unknown;
                        right_elbow_general_state = JointGeneralState.Unknown;

                        // Reset the stopwatch:
                        stopWatch_trigger.Reset();
                    }
                }
            }

            TextInformation.insert_main_text_block("Event: " + stopWatch_single_event.Elapsed, 3);
            TextInformation.update_main_text();
        }

    }

    public static class GestureAuxilaryMethods
    {
        public static Vector3D updateLimbVectors(JointType joint_start_type, JointType joint_end_type)
        {
            Vector3D ret_vector = new Vector3D();
            Joint joint_start = GesturesMasterControl.body.Joints[joint_start_type];
            Joint joint_end = GesturesMasterControl.body.Joints[joint_end_type];

            ret_vector.X = joint_end.Position.X - joint_start.Position.X;
            ret_vector.Y = joint_end.Position.Y - joint_start.Position.Y;
            ret_vector.Z = joint_end.Position.Z - joint_start.Position.Z;

            return ret_vector;
        }

        public static bool isJointStable(double sensitivity, JointType joint, Joint[] prev_array, int prev_length, ref JointGeneralState elbow_state)
        {
            bool return_bool = false;
            //double avg_standard_dev = calcAverageStandardDev(prev_array, prev_length);

            //if (avg_standard_dev < sensitivity)
            if (calcFirstLastDiff(prev_array, prev_length, sensitivity))
            {
                return_bool = true;
                elbow_state = JointGeneralState.Still;
                TextInformation.insert_main_text_block(joint + " Still", 2);
            }
            else
            {
                return_bool = false;
                elbow_state = JointGeneralState.Moving;
                TextInformation.insert_main_text_block(joint + " Moving", 2);
            }

            return return_bool;
        }

        private static bool calcFirstLastDiff(Joint[] joint_array, int prev_length, double sensitivity)
        {
            double[] x_positions = new double[2];
            double[] y_positions = new double[2];
            double[] z_positions = new double[2];

            // Getting the position for first and last element:
            x_positions[0] = joint_array[0].Position.X;
            y_positions[0] = joint_array[0].Position.Y;
            z_positions[0] = joint_array[0].Position.Z;
            x_positions[1] = joint_array[prev_length - 1].Position.X;
            y_positions[1] = joint_array[prev_length - 1].Position.Y;
            z_positions[1] = joint_array[prev_length - 1].Position.Z;

            if (Math.Abs(x_positions[0] - x_positions[1]) < sensitivity &&
               Math.Abs(y_positions[0] - y_positions[1]) < sensitivity &&
               Math.Abs(z_positions[0] - z_positions[1]) < sensitivity)
            {
                return true;
            }
            else
            {
                return false;
            }
        }

        private static double calcAverageStandardDev(Joint[] joint_array, int prev_length)
        {
            float[] x_positions = new float[prev_length];
            float[] y_positions = new float[prev_length];
            float[] z_positions = new float[prev_length];
            double x_average = 0;
            double y_average = 0;
            double z_average = 0;
            double x_dev_numerator = 0;
            double y_dev_numerator = 0;
            double z_dev_numerator = 0;
            double x_intermed = 0;
            double y_intermed = 0;
            double z_intermed = 0;
            double avg_standard_dev = 0;
            int i = 0;

            // Getting the position from joints and filling up respective arrays:
            for (i = 0; i < prev_length; i++)
            {
                x_positions[i] = joint_array[i].Position.X;
                y_positions[i] = joint_array[i].Position.Y;
                z_positions[i] = joint_array[i].Position.Z;
            }

            // Calculating averages:
            for (i = 0; i < prev_length; i++)
            {
                x_average += x_positions[i];
                y_average += y_positions[i];
                z_average += z_positions[i];
            }

            x_average /= prev_length;
            y_average /= prev_length;
            z_average /= prev_length;

            // Calculating the numerators:
            for (i = 0; i < prev_length; i++)
            {
                x_intermed = x_positions[i] - x_average;
                x_dev_numerator += x_intermed * x_intermed;
                y_intermed = y_positions[i] - y_average;
                y_dev_numerator += y_intermed * y_intermed;
                z_intermed = z_positions[i] - z_average;
                z_dev_numerator += z_intermed * z_intermed;
            }

            x_dev_numerator /= x_average;
            y_dev_numerator /= y_average;
            z_dev_numerator /= z_average;

            x_dev_numerator = Math.Sqrt(Math.Abs(x_dev_numerator));
            y_dev_numerator = Math.Sqrt(Math.Abs(y_dev_numerator));
            z_dev_numerator = Math.Sqrt(Math.Abs(z_dev_numerator));

            avg_standard_dev = (x_dev_numerator + y_dev_numerator + z_dev_numerator) / 3;
            return avg_standard_dev;
        }

        public static void managePrevArray(Joint joint, ref Joint[] array, int prev_length)
        {
            int i = prev_length - 1;
            while (i > 0)
            {
                array[i] = array[i - 1];
                i--;
            }
            array[i] = joint;
        }
    }

    public static class TextInformation
    {
        public static TextBlock main_text_block_1 = new TextBlock();
        public static TextBlock main_text_block_2 = new TextBlock();
        public static TextBlock main_text_block_3 = new TextBlock();
        public static TextBlock main_text_block_4 = new TextBlock();

        public static String[,] main_text = new String[4, 18];

        private static int[] lines = { 0, 0, 0, 0 };
        private static bool[] has_block_changed = { false, false, false, false };

        public static void insert_main_text_block(String info, int block)
        {
            main_text[block - 1, lines[block - 1]++] = info;
            has_block_changed[block - 1] = true;
        }

        public static void reset_text()
        {
            main_text_block_1.Text = " ";
            main_text_block_2.Text = " ";
            main_text_block_3.Text = " ";
            main_text_block_4.Text = " ";
        }

        public static void update_main_text()
        {
            int i = 0, j = 0;

            for (i = 0; i < 4; i++)
            {
                if (has_block_changed[i] == true)
                {

                    String y = "";
                    // 18 lines can fit:
                    for (j = 0; j < 18; j++)
                    {
                        y += main_text[i, j] + "\n";
                    }

                    if (i == 0)
                    {
                        main_text_block_1.Text = y;
                    }
                    else if (i == 1)
                    {
                        main_text_block_2.Text = y;
                    }
                    else if (i == 2)
                    {
                        main_text_block_3.Text = y;
                    }
                    else
                    {
                        main_text_block_4.Text = y;
                    }

                }

                has_block_changed[i] = false;
                lines[i] = 0;
            }

        }
    }
}
