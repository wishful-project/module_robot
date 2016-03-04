import logging
import random
import wishful_upis as upis
import wishful_agent as wishful_module
from ws4py.client.threadedclient import WebSocketClient

__author__ = "Johannes Kunkel"
__copyright__ = "Copyright (c) 2015, Technische Universität Berlin"
__version__ = "0.1.0"
__email__ = "johannes.kunkel@campus.tu-berlin.de"


@wishful_module.build_module
class RobotModule(wishful_module.AgentModule):
    def __init__(self):
        super(RobotModule, self).__init__()
        self.log = logging.getLogger('robot_module.main')
        self.ros_client = None

    @wishful_module.bind_function(upis.context.set_position)
    def set_position(self, position, orientation):
        self.log.debug("Robot Module sets pose {}: {}".format((position,orientation), self.setPose))
        self.channel = channel
        return None

    @wishful_module.bind_function(upis.context.get_position)
    def get_position(self):
        self.log.debug("Robot Module gets position: {}".format(self.getPose)
        return ros_client.getPose()

    @wishful_module.bind_function(upis.context.set_host)
    def set_host(self, host):
        self.log.debug("Robot Module sets ROS host {}: {}".format(host, self.set_host)

        if (not (self.ros_client is None)):
            self.ros_client.stop()

        self.ros_client = ThreadedPositionClient(host, self.log)

        self.ws_client.connect()
        return

class ThreadedPositionClient(WebSocketClient):
    def __init__(self, host, log):
        WebSocketClient.__init__(self,host)

        #buffer for incoming messages
        self.buff=''
        #the latest full position message that was received is saved here for easy reference
        self.last_message=''
        #lock for getPose access
        self.pl_lock = threading.Lock()

        self.log=log

    def opened(self):
        #subscribe to topic
        msg={'op': 'subscribe', 'topic': '/twistbot/info/position'}

        self.send(json.dumps(msg)) 

    def closed(self, code, reason):
        log.warning('Websocket closed down. (code ' + code + ', ' + reason + ')')

    def stop(self):
        WebSocketClient.close(self, reason='Connection closed.')

    def received_message(self, m):
        if m.is_text:
            m=m.data.decode('utf-8')
        else:
            return

        self.buff+=m
        
        #parse_buffer until it returns None
        while True:
            parsed_msg=self.parse_buffer()
            if not parsed_msg is None:
                self.pl_lock.acquire()
                self.last_message=parsed_msg
                self.pl_lock.release()
            else:
                break

    def parse_buffer(self):
        #Find matching pairs of brackets that demark start and end of a message
        num_opened=0
        num_closed=0
        idx_start=-1
        idx_end=-1

        for i in range(0,len(self.buff)):

            if self.buff[i]=='{':
                if idx_start==-1:
                    idx_start=i

                num_opened+=1

            else:
                if self.buff[i]=='}':
                    num_closed+=1

            if num_opened>0 and num_opened==num_closed:
                idx_end=i+1
                break

        #if no full message was found, return
        if idx_end==-1 or idx_start==-1:
            return None

        #split the msg off the buffer
        msg=self.buff[idx_start:idx_end]
        self.buff=self.buff[idx_end:]

        return str(msg)

    def buildPose(pos,rot):
        pose='position: {x: '+pos[0]
        pose+=', y: '+pos[1]
        pose+=', z: '+pos[2]+'}, '

        pose+='orientation: {w: '+rot[0]
        pose+=', x: '+rot[0]
        pose+=', y: '+rot[1]
        pose+=', z: '+rot[2]+'}'
        return pose

    def setPose(self, position, orientation=(1,0,0,0)):
        header='frame_id: "map"'

        pose=buildPose(position,orientation)

        msg_string='{header: {'+header+'}, pose: {'+pose+'}}'

        msg={'op': 'publish', 'topic': '/move_base_simple/goal', 'msg':msg_string}

        self.send(json.dumps(msg))

    def getPose(self):
        self.pl_lock.acquire()

        if self.last_message == '':
            return None

        dct=json.loads(self.last_message)

        self.pl_lock.release()

        #takes a string and converts it to pose info
        pose=dct[u'msg'][u'pose']
        position=pose[u'position']
        orientation=pose[u'orientation']

        #Now convert these to their fully parsed equivalents
        position=(float(position[u'x']),\
                  float(position[u'y']),\
                  float(position[u'z']))
        orientation=(float(orientation[u'w']),\
                     float(orientation[u'x']),\
                     float(orientation[u'y']),\
                     float(orientation[u'z']))

        return position, orientation