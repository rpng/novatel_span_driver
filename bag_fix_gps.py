#!/usr/bin/env python
import rosbag
import os
import sys
import time
import signal
import math
import functools
import argparse
import threading
import multiprocessing
from multiprocessing import Pool

# Create our argument parser
parser = argparse.ArgumentParser(description='Converts bag gps covariance values' 
                   'to the correct sigma squares. Original novatel span driver'
                   'had 2^std_dev instead of std_dev^2.')
parser.add_argument('bagfiles', metavar='bagfiles', type=str, nargs='+',
                   help='The list of bagfiles that should be parsed')
parser.add_argument('-tg', 
                    dest='topic_gps', 
                    metavar='topic_gps',
                    action='store',
                    default='/navsat/fix',
                    help='The NavSatFixed topic that should be corrected.')
parser.add_argument('-to', 
                    dest='topic_odom', 
                    metavar='topic_odom',
                    action='store',
                    default='/navsat/odom',
                    help='The Odometry topic that should be corrected.')


#######################################################
#######################################################
#######################################################


# Recursive set attribute function
# https://stackoverflow.com/a/31174427/7718197
def rsetattr(obj, attr, val):
    pre, _, post = attr.rpartition('.')
    return setattr(rgetattr(obj, pre) if pre else obj, post, val)

# Recursive get attribute function
# https://stackoverflow.com/a/31174427/7718197
sentinel = object()
def rgetattr(obj, attr, default=sentinel):
    if default is sentinel:
        _getattr = getattr
    else:
        def _getattr(obj, name):
            return getattr(obj, name, default)
    return functools.reduce(_getattr, [obj]+attr.split('.'))

#######################################################
#######################################################
#######################################################


def process_bag(bagfile):
    try:
        # Debug messages
        print("Opening: %s" % bagfile)
        # Start timer
        start_time = time.time()
        # Get the name of the bag
        filename, fileext = os.path.splitext(bagfile);
        # Open new and old bag file
        bagfile_old = rosbag.Bag(bagfile)
        bagfile_new = rosbag.Bag(filename+"_fixed"+fileext, 'w')
        # Get bag stats
        ct_curr = 0;
        ct_total = bagfile_old.get_message_count();
        print("Total Messages: %d" % ct_total)
        # Loop through all messages in the bag
        for topic, msg, timestamp in bagfile_old.read_messages():
            # If our topic is for GPS then correct that covariance
            if topic == args['topic_gps']:
                # Get the covariance, and convert it
                # Note that the system originaly used value = 2^std_dev
                # Thus we can find that std_dev = log(value)/log(2) y>0
                cov = list(rgetattr(msg,"position_covariance"))
                cov[0] = pow(math.log10(cov[0])/math.log10(2),2);
                cov[4] = pow(math.log10(cov[4])/math.log10(2),2);
                cov[8] = pow(math.log10(cov[8])/math.log10(2),2);
                # Update the covariance
                rsetattr(msg,"position_covariance",cov)
            # If the topic is for ODOM then correct that covariance
            if topic == args['topic_odom']:
                # Get the covariance, and convert it
                # Note that the system originaly used value = 2^std_dev
                # Thus we can find that std_dev = log(value)/log(2) y>0
                cov_pose = list(rgetattr(msg,"pose.covariance"))
                cov_twist = list(rgetattr(msg,"twist.covariance"))
                cov_pose[21] = pow(math.log10(cov_pose[21])/math.log10(2),2);
                cov_pose[28] = pow(math.log10(cov_pose[28])/math.log10(2),2);
                cov_pose[35] = pow(math.log10(cov_pose[35])/math.log10(2),2);
                cov_twist[0] = pow(math.log10(cov_twist[0])/math.log10(2),2);
                cov_twist[7] = pow(math.log10(cov_twist[7])/math.log10(2),2);
                cov_twist[14] = pow(math.log10(cov_twist[14])/math.log10(2),2);
                # Update the covariances
                rsetattr(msg,"pose.covariance",cov_pose)
                rsetattr(msg,"twist.covariance",cov_twist)
            # Finally write all messages into the new bag file
            bagfile_new.write(topic,msg,timestamp)
        # Finally done with the bags, so close them
        bagfile_old.close()
        bagfile_new.close()
        # Debug print
        end_time = time.time()
        print("\033[92mDone: %g seconds \033[0m" % (end_time - start_time))
    except KeyboardInterrupt:
        return
    except Exception as e:
        print("\033[91mError: %s \033[0m" % bagfile)
        print("\033[91mError: %s \033[0m" % e)
        return


#######################################################
#######################################################
#######################################################

# Parse our passed arguments
args = vars(parser.parse_args())

# Number of processors
numProcesses = max(1,multiprocessing.cpu_count()-1)

try:
    # Loop through each bag file
    sigint_handler = signal.signal(signal.SIGINT, signal.SIG_IGN)
    pool = multiprocessing.Pool(numProcesses)
    signal.signal(signal.SIGINT, sigint_handler)
    pool.map(process_bag, args['bagfiles'])
    pool.close()
except KeyboardInterrupt:
    print("Stopping all threads...")
    pool.terminate()
    pool.join()
finally:
    # Make sure we close everything
    pool.close()
    pool.join()