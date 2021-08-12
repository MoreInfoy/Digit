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
x_last = 0
y_last = 0


def my_handler(channel, data):
    global t_last
    global x_last
    global y_last
    msg = RobotMessage.decode(data)
    """ print("Received message on channel \"%s\"" % channel)
    print("   lfWrench = %s" % str(msg.lfWrench))
    print("   rfWrench = %s" % str(msg.rfWrench))
    print("   lfWrench_des = %s" % str(msg.lfWrench_des))
    print("   position = %s" % str(msg.rfWrench_des)) """
    # plt.plot(msg.timeStamp, msg.data[2], 'k*')
    index = 0
    # plt.plot([t_last, msg.timeStamp], [x_last, msg.data[index]], 'k-')
    # plt.plot([t_last, msg.timeStamp], [y_last, msg.data[index + 3]], 'r-')

    plt.plot(msg.data[0::6], 'k-')
    plt.plot(msg.data[1::6], 'r-')
    plt.plot(msg.data[2::6], 'b-')

    plt.plot(msg.data[3::6], 'k--')
    plt.plot(msg.data[4::6], 'r--')
    plt.plot(msg.data[5::6], 'b--')

    # plt.plot([t_last, msg.timeStamp], [x_last, msg.data[index]], 'k-', [t_last, msg.timeStamp],
    #          [y_last, msg.data[index + 3]], 'r-')

    t_last = msg.timeStamp
    x_last = msg.data[index]
    y_last = msg.data[index + 1]
    plt.pause(0.0001)
    plt.clf()


if __name__ == '__main__':
    lc = lcm.LCM()
    subscription = lc.subscribe(topic, my_handler)
    try:
        while True:
            lc.handle()
    except KeyboardInterrupt:
        pass
