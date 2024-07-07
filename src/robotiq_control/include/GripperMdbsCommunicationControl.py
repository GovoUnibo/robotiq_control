from robotiq_control.include.GripperModbusRtu import RobotiqCommunication, Robotiq
from robotiq_control.include.GripperCommon import RobotiqGripperType
from threading import Thread
import warnings
import time

class GripperCommand(RobotiqCommunication):
    def __init__(self, gripper_type :str, id=0, comPort='/dev/ttyUSB0',baud_rate=115200, timeout=0.002, stroke=None):
        
        if not gripper_type in ['2F_85', 'Hand_E']:
            raise ValueError("Gripper type must be '2F_85' or 'Hand_E'")
        
        if gripper_type == '2F_85':
            gripper_type = RobotiqGripperType.TwoF_85
        elif gripper_type == 'Hand_E':
            gripper_type = RobotiqGripperType.Hand_E
            
        RobotiqCommunication.__init__(self, gripper_type=gripper_type, device_id=id, com_port=comPort, baud=baud_rate, timeout=timeout, stroke=stroke)

        self._max_stroke = self.max_stroke   #super().max_stroke
        self._min_stroke = self.min_stroke   #super().min_stroke
        self._max_grasp_force = self.max_grasp_force
        self.gripper_stroke = self.stroke
        self.time_expired = False
        sec_before_expire = 2
        self.threadTimer = Thread(target=self.__internalCountDown_sec, args=(sec_before_expire,))
        self.threadCheckConnection = Thread(target=self.__CheckConnection)
        self.is_gripper_connected = False

        self.__debug_ = False

    def __internalCountDown_sec(self, seconds):
        #print("Started Countdown of ", seconds, "seconds")
        start = round(time.perf_counter(), 3)
        while ( (round(time.perf_counter(), 3) - start) < seconds):
            self.time_expired = False
        self.time_expired = True
    
    def __CheckConnection(self):
        self.is_gripper_connected = self.checkConnection()
        if self.is_gripper_connected:
            print('Connection Active')
        else:
            print('Connection Lost')
        time.sleep(1)

    def initialize(self):
        
        if self.gripperConnect():
            self.is_gripper_connected = True
        
        self.deactivate_gripper()
        
        if self.is_reset():
            
            print("Activation Request\n")
            self.activate_gripper()
            time.sleep(3)
        
        if self.is_ready():
            print("Gripper is Rdy")
            return True
        else:
            return False
        
    def _clamp_position(self,pos):
        out_of_bouds = False
        if (pos <= self._min_stroke):
            out_of_bouds = True
            pos_corrected = self._min_stroke
        elif (pos > self._max_stroke):
            out_of_bouds = True
            pos_corrected = self._max_stroke
        if(out_of_bouds):
            #print in yellow
            print("\033[93mPosition (%.3f[m]) out of limits for %d[mm] gripper: \n- New position: %.3f[m]\n- Min position: %.3f[m]\n- Max position: %.3f[m] \033[0m" % (pos, self._max_stroke, pos_corrected, self._min_stroke, self._max_stroke))
            # warnings.warn("Position (%.3f[m]) out of limits for %d[mm] gripper: \n- New position: %.3f[m]\n- Min position: %.3f[m]\n- Max position: %.3f[m]" % (pos, self._max_stroke, pos_corrected, self._min_stroke, self._max_stroke))
            pos = pos_corrected
        return pos

    def _clamp_speed(self,vel):
        out_of_bouds = False
        if (vel <= 0.013):
            out_of_bouds = True
            vel_corrected = 0.013
        elif (vel > 0.101):
            out_of_bouds = True
            vel_corrected = 0.1
        if(out_of_bouds):
            print("\033[93mSpeed (%.3f[m/s]) out of limits: \n- New speed: %.3f[m/s]\n- Min speed: %.3f[m/s]\n- Max speed: %.3f[m/s] \033[0m" % (vel, vel_corrected, 0.013, 0.1))
            vel = vel_corrected
        return vel
    
    def _clamp_force(self,force):
        out_of_bouds = False
        if (force < 0.0):
            out_of_bouds = True
            force_corrected = 0.0
        elif (force > 100.0):
            out_of_bouds = True
            force_corrected = 100.0
        if(out_of_bouds):
            # rospy.logdebug("Force (%.3f[%]) out of limits for %d[mm] gripper: \n- New force: %.3f[%]\n- Min force: %.3f[%]\n- Max force: %.3f[%]" % (force, self._gripper.stroke*1000, force_corrected, 0, 100))
            force = force_corrected
        return force

    def getGipperStatus(self):
        
        feedback = []
        feedback.append(self.is_ready())
        feedback.append(self.is_reset())
        feedback.append(self.is_moving())
        feedback.append(self.object_detected())
        feedback.append(self.get_fault_status())
        feedback.append(self.get_pos())
        feedback.append(self.get_req_pos())
        feedback.append(self.get_current())

        return feedback

    def goTo(self, pos, speed, force):
        pos     = self._clamp_position(pos)
        speed   = self._clamp_speed(speed)
        force   = self._clamp_force(force)
        send_success = self.sendUnmonitoredMotionCmd(pos, speed, force)
        if self.__debug_ :
            if send_success :
                print('Cmd Sent')
            else:
                print('Cmd Not Sent')

        return send_success
        
    def open_(self, speed=0.1, force=100):
        return self.sendUnmonitoredMotionCmd(self._max_stroke, speed, force)

    def close_(self, speed=0.1, force=100):
        return self.sendUnmonitoredMotionCmd(self._min_stroke, speed, force)
        #attenzione il metodo close() è ereitato dalla classe che gestisce la comunicazione mdbs con il gripper


if __name__ == "__main__":
    from serial.tools import list_ports as readComPorts
    import os, sys
    sys.path.append(os.path.join(os.path.dirname(__file__)))
    # gripperComm = GripperCommand(gripper_type='Hand_E', id=0, comPort='/dev/ttyUSB0', baud_rate=115200, timeout=0.002)
    # threadMonitorGripperStatus = Thread(target=gripperComm.getGipperStatus())

    # def checkPresenceComPort(COM_port):
    #     myports = list(readComPorts.comports())
    #     for port in myports:
    #         if COM_port == port.device:
    #             return True

    #     return False

    # def readPorts():
    #     myports = list(readComPorts.comports())
    #     list_of_ports = []
    #     for port in myports:
    #         list_of_ports.append(port.device)
    #     return list_of_ports

    # Port = '/dev/ttyUSB0'
    # print("List of Ports: ", readPorts())
    # if not checkPresenceComPort(Port):
    #     print("Default '{}' COM Port not found".format(Port))
    #     Port = input("Insert COM Port: ")

    
    # gripper_init = False
    # while not gripper_init:
    #     print("Waiting for gripper to be ready...")
    #     gripper_init = gripperComm.initialize()
    #     time.sleep(0.5)
    #     exit()
    
    # gripperComm.goTo(0.055, 0.1, 100)