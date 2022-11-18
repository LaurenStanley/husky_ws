import rosbags.rosbag2

def extract_chunks(file_in, chunks):
    bagfile = rosbag.Bag(file_in)
    messages = bagfile.get_message_count()
    m_per_chunk = int(round(float(messages) / float(chunks)))
    chunk = 0
    m = 0
    outbag = rosbag.Bag("chunk_%04d.bag" % chunk, 'w')
    for topic, msg, t in bagfile.read_messages():
        m += 1
        if m % m_per_chunk == 0:
            outbag.close()
            chunk += 1
            outbag = rosbag.Bag("chunk_%04d.bag" % chunk, 'w')
        outbag.write(topic, msg, t)
    outbag.close()


