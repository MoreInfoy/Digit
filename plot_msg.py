import lcm
from User.include.lcm.RobotMessage import RobotMessage
import matplotlib.pyplot as plt

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
    plt.plot([t_last, msg.timeStamp], [x_last, msg.data[index]], 'k-')
    plt.plot([t_last, msg.timeStamp], [y_last, msg.data[index+1]], 'r-')

    # plt.plot([t_last, msg.timeStamp], [x_last, msg.data[index]], 'k-', [t_last, msg.timeStamp], [y_last, msg.data[index+3]], 'r-')
    
    t_last = msg.timeStamp
    x_last = msg.data[index]
    y_last = msg.data[index+1]
    plt.pause(0.0001)


if __name__ == '__main__':
    lc = lcm.LCM()
    subscription = lc.subscribe(topic, my_handler)
    try:
        while True:
            lc.handle()
    except KeyboardInterrupt:
        pass
