#####
# The awesome original code of this is here : github.com/AtsushiSakai/rosbag_to_csv
#####

import os
import rosbag
import rospy
import sys
from datetime import datetime

def enum_file_name(path="./"):
    files = []
    current_list = os.listdir(path)

    for elem in current_list:
        full_path = path + "/" + elem

        if os.path.isdir(full_path):
            files.extend(enum_file_name(path=full_path))
        elif ".DS_Store" in full_path:
            continue
        else:
            files.append(full_path)
    return files

def make_dir_to_path(path):
    path_dir = os.path.dirname(path)
    if not os.path.exists(path_dir):
        os.makedirs(path_dir)

def message_to_csv(stream, msg, filter=[], flatten=False):
    try:
        for s in type(msg).__slots__:
            if s in filter:
                val = msg.__getattribute__(s)
                message_to_csv(stream, val, flatten)
    except:
        msg_str = str(msg)
        if msg_str.find(",") is not -1:
            msg_str = msg_str.strip("(")
            msg_str = msg_str.strip(")")
            msg_str = msg_str.strip(" ")
            msg_str = msg_str.strip("(")
        stream.write("," + msg_str)


def message_type_to_csv(stream, msg, parent_content_name=""):
    """
    stream: StringIO
    msg: message
    """
    try:
        for s in type(msg).__slots__:
            val = msg.__getattribute__(s)
            message_type_to_csv(stream, val, ".".join([parent_content_name, s]))
    except:
        stream.write("," + parent_content_name)



def bag_to_csv(rosbag_file, csv_folder, topic_names, output_file_format=".csv", filter="position", header=False):
    try:
        bag = rosbag.Bag(rosbag_file)
        streamdict = dict()
    except Exception as e:
        rospy.logfatal('failed to load bag file: %s', e)
        exit(1)

    try:
        for topic, msg, time in bag.read_messages(topics=topic_names):
            # print msg
            if streamdict.has_key(topic):
                stream = streamdict[topic]
            else:
                rosbag_file = rosbag_file.replace(os.path.dirname(rosbag_file), csv_folder)
                print rosbag_file
                rosbag_file = rosbag_file.replace('.bag', output_file_format)

                print rosbag_file

                make_dir_to_path(rosbag_file)
                stream = open(rosbag_file, 'w')
                streamdict[topic] = stream
                # header
                if header:
                    stream.write("time")
                    message_type_to_csv(stream, msg)
                    stream.write('\n')

            stream.write(datetime.fromtimestamp(time.to_time()).strftime('%S.%f'))
            message_to_csv(stream, msg, filter=[filter], flatten=not header)
            stream.write('\n')
        [s.close for s in streamdict.values()]
    except Exception as e:
        rospy.logwarn("fail: %s", e)
    finally:
        bag.close()


if __name__ == '__main__':
    argvs = sys.argv
    argc = len(argvs)

    rosbag_folder_name = argvs[1]
    csv_folder_name = argvs[2]

    if argc <= 2:
        print 'Usage: $ python converter.py (rosbag_folder_path) (new_folder_name)'

    filename_list = enum_file_name(argvs[1])
    for filename in filename_list:
        if ".bag" in filename:
            print filename
            bag_to_csv(rosbag_file=filename, csv_folder=csv_folder_name,
                       topic_names=["/ftsensor_raw", "/ftsensor_mean"], filter="data", header=False)
