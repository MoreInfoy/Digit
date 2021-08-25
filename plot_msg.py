import lcm
import matplotlib.pyplot as plt

try:
    import cStringIO.StringIO as BytesIO
except ImportError:
    from io import BytesIO
import struct


class RobotMessage(object):
    __slots__ = ["timeStamp", "data_size", "data"]

    def __init__(self):
        self.timeStamp = 0.0
        self.data_size = 0
        self.data = []

    def encode(self):
        buf = BytesIO()
        buf.write(RobotMessage._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack(">di", self.timeStamp, self.data_size))
        buf.write(struct.pack('>%dd' % self.data_size, *self.data[:self.data_size]))

    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != RobotMessage._get_packed_fingerprint():
            raise ValueError("Decode error")
        return RobotMessage._decode_one(buf)

    decode = staticmethod(decode)

    def _decode_one(buf):
        self = RobotMessage()
        self.timeStamp, self.data_size = struct.unpack(">di", buf.read(12))
        self.data = struct.unpack('>%dd' % self.data_size, buf.read(self.data_size * 8))
        return self

    _decode_one = staticmethod(_decode_one)

    _hash = None

    def _get_hash_recursive(parents):
        if RobotMessage in parents: return 0
        tmphash = (0x91bb7785ac915e68) & 0xffffffffffffffff
        tmphash = (((tmphash << 1) & 0xffffffffffffffff) + (tmphash >> 63)) & 0xffffffffffffffff
        return tmphash

    _get_hash_recursive = staticmethod(_get_hash_recursive)
    _packed_fingerprint = None

    def _get_packed_fingerprint():
        if RobotMessage._packed_fingerprint is None:
            RobotMessage._packed_fingerprint = struct.pack(">Q", RobotMessage._get_hash_recursive([]))
        return RobotMessage._packed_fingerprint

    _get_packed_fingerprint = staticmethod(_get_packed_fingerprint)


plt.ion()
plt.figure(1)

topic = "ROBOT_MESSAGE_TOPIC"

t_last = 0
x1_last = 0
y1_last = 0
x2_last = 0
y2_last = 0
x3_last = 0
y3_last = 0

lx1_last = 0
ly1_last = 0
lx2_last = 0
ly2_last = 0
lx3_last = 0
ly3_last = 0

rx1_last = 0
ry1_last = 0
rx2_last = 0
ry2_last = 0
rx3_last = 0
ry3_last = 0


def my_handler(channel, data):
    global t_last
    global x1_last
    global x2_last
    global x3_last
    global y1_last
    global y2_last
    global y3_last

    global lx1_last
    global lx2_last
    global lx3_last
    global ly1_last
    global ly2_last
    global ly3_last

    global rx1_last
    global rx2_last
    global rx3_last
    global ry1_last
    global ry2_last
    global ry3_last

    msg = RobotMessage.decode(data)
    """ print("Received message on channel \"%s\"" % channel)
    print("   lfWrench = %s" % str(msg.lfWrench))
    print("   rfWrench = %s" % str(msg.rfWrench))
    print("   lfWrench_des = %s" % str(msg.lfWrench_des))
    print("   position = %s" % str(msg.rfWrench_des)) """
    # plt.plot(msg.timeStamp, msg.data[2], 'k*')
    # index = 0
    # plt.plot([t_last, msg.timeStamp], [x_last, msg.data[index]], 'r-')
    # plt.plot([t_last, msg.timeStamp], [y_last, msg.data[index + 3]], 'r--')

    # planning com trajectory
    # plt.plot(msg.data[0::8], 'r-')
    # plt.plot(msg.data[1::8], 'g-')
    # plt.plot(msg.data[2::8], 'b-')
    #
    # plt.plot(msg.data[3::8], 'r--')
    # plt.plot(msg.data[4::8], 'g--')
    # plt.plot(msg.data[5::8], 'b--')
    #
    # plt.plot(msg.data[6::8], 'k-*')
    # plt.plot(msg.data[7::8], 'c-*')

    plt.subplot(6, 1, 1)
    plt.plot([t_last, msg.timeStamp], [lx1_last, msg.data[0]], 'r-')
    plt.plot([t_last, msg.timeStamp], [ly1_last, msg.data[3]], 'r--')
    # plt.xlim(msg.timeStamp - 3, msg.timeStamp + 0.1)

    plt.subplot(6, 1, 2)
    plt.plot([t_last, msg.timeStamp], [lx2_last, msg.data[1]], 'g-')
    plt.plot([t_last, msg.timeStamp], [ly2_last, msg.data[4]], 'g--')
    # plt.xlim(msg.timeStamp - 3, msg.timeStamp + 0.1)

    plt.subplot(6, 1, 3)
    plt.plot([t_last, msg.timeStamp], [lx3_last, msg.data[2]], 'b-')
    plt.plot([t_last, msg.timeStamp], [ly3_last, msg.data[5]], 'b--')
    # plt.xlim(msg.timeStamp - 3, msg.timeStamp + 0.1)

    plt.subplot(6, 1, 4)
    plt.plot([t_last, msg.timeStamp], [rx1_last, msg.data[6]], 'r-')
    plt.plot([t_last, msg.timeStamp], [ry1_last, msg.data[9]], 'r--')
    # plt.xlim(msg.timeStamp - 3, msg.timeStamp + 0.1)

    plt.subplot(6, 1, 5)
    plt.plot([t_last, msg.timeStamp], [rx2_last, msg.data[7]], 'g-')
    plt.plot([t_last, msg.timeStamp], [ry2_last, msg.data[10]], 'g--')
    # plt.xlim(msg.timeStamp - 3, msg.timeStamp + 0.1)

    plt.subplot(6, 1, 6)
    plt.plot([t_last, msg.timeStamp], [rx3_last, msg.data[8]], 'b-')
    plt.plot([t_last, msg.timeStamp], [ry3_last, msg.data[11]], 'b--')
    # plt.xlim(msg.timeStamp - 3, msg.timeStamp + 0.1)

    t_last = msg.timeStamp
    lx1_last = msg.data[0]
    ly1_last = msg.data[3]
    lx2_last = msg.data[1]
    ly2_last = msg.data[4]
    lx3_last = msg.data[2]
    ly3_last = msg.data[5]

    rx1_last = msg.data[6]
    ry1_last = msg.data[9]
    rx2_last = msg.data[7]
    ry2_last = msg.data[10]
    rx3_last = msg.data[8]
    ry3_last = msg.data[11]

    # com trajectory
    # plt.subplot(3, 1, 1)
    # plt.plot([t_last, msg.timeStamp], [x1_last, msg.data[0]], 'r-')
    # plt.plot([t_last, msg.timeStamp], [y1_last, msg.data[3]], 'r--')
    # plt.xlim(msg.timeStamp - 3, msg.timeStamp + 0.1)
    #
    # plt.subplot(3, 1, 2)
    # plt.plot([t_last, msg.timeStamp], [x2_last, msg.data[1]], 'g-')
    # plt.plot([t_last, msg.timeStamp], [y2_last, msg.data[4]], 'g--')
    # plt.xlim(msg.timeStamp - 3, msg.timeStamp + 0.1)
    #
    # plt.subplot(3, 1, 3)
    # plt.plot([t_last, msg.timeStamp], [x3_last, msg.data[2]], 'b-')
    # plt.plot([t_last, msg.timeStamp], [y3_last, msg.data[5]], 'b--')
    # plt.xlim(msg.timeStamp - 3, msg.timeStamp + 0.1)
    #
    # t_last = msg.timeStamp
    # x1_last = msg.data[0]
    # y1_last = msg.data[3]
    # x2_last = msg.data[1]
    # y2_last = msg.data[4]
    # x3_last = msg.data[2]
    # y3_last = msg.data[5]

    # joint torque
    # plt.subplot(2, 1, 1)
    # plt.plot([t_last, msg.timeStamp], [x1_last, msg.data[0]], 'r-')
    # plt.xlim(msg.timeStamp - 1.5, msg.timeStamp + 0.1)
    #
    # plt.subplot(2, 1, 2)
    # plt.plot([t_last, msg.timeStamp], [y1_last, msg.data[1]], 'g-')
    # plt.xlim(msg.timeStamp - 1.5, msg.timeStamp + 0.1)
    # t_last = msg.timeStamp
    # x1_last = msg.data[0]
    # y1_last = msg.data[1]

    # plt.legend(['xpos', 'xvel', 'xacc', 'ypos', 'yvel', 'yacc'], loc='upper right')
    # plt.ylim(-1.0, 1.0)
    plt.pause(1e-12)
    # plt.clf()
    # plt.xlim(msg.timeStamp - 10, msg.timeStamp + 0.1)


if __name__ == '__main__':
    lc = lcm.LCM()
    subscription = lc.subscribe(topic, my_handler)
    try:
        while True:
            lc.handle()
    except KeyboardInterrupt:
        pass
