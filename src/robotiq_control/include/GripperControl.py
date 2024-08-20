import os, sys
sys.path.append(os.path.join(os.path.dirname(__file__)))
from GripperMdbsCommunicationControl import GripperCommand
from GripperTcpIpCommControl import GripperSocket


class GripperControl:
    def __init__(self, gripper_type: str, commu_type: str, **kwargs) -> None:

        if not gripper_type in ['2F_85', 'Hand_E']:
            raise ValueError("Gripper type must be '2F_85' or 'Hand_E'")

        
        if not commu_type in ['tcpip', 'modbus']:
            raise ValueError("Communication type must be 'tcpip' or 'modbus'")
        


        if commu_type == 'tcpip':
            self.gripper = GripperSocket(robot_ip=kwargs.get('robot_ip', '192.168.0.102'),
                                              port=kwargs.get('port', 63352),
                                              gripper_type=gripper_type,
                                              stroke=kwargs.get('stroke', None)
                                        )
        elif commu_type == 'modbus':
            self.gripper = GripperCommand(gripper_type=gripper_type,
                                               id=kwargs.get('id', 0),
                                               comPort=kwargs.get('comPort', '/dev/ttyUSB0'),
                                               baud_rate=kwargs.get('baud_rate', 115200),
                                               timeout=kwargs.get('timeout', 0.002),
                                               stroke=kwargs.get('stroke', None)
                                            )
    
    def set_min_stroke(self, min_stroke: float) -> None:
        self.gripper.set_min_stroke(min_stroke)
    
    def set_max_stroke(self, max_stroke: float) -> None:
        self.gripper.set_max_stroke(max_stroke)
    
    def set_stroke(self, stroke: float) -> None:
        self.gripper.set_stroke(stroke)

    def get_stroke(self) -> float:
        return self.gripper.get_stroke()

    def get_max_stroke(self) -> float:
        return self.gripper.max_stroke

    def get_min_stroke(self) -> float:
        return self.gripper.min_stroke

    def set_max_force(self, max_force: float) -> None:
        self.gripper.set_max_force(max_force)
        
    def get_max_force(self) -> float:
        return self.gripper.get_max_force()

    def get_status(self, as_dict=False) -> dict:
        '''
        Return the status of the gripper if as_dict is True, otherwise return the status as a list
        '''
        if not as_dict:
            return self.gripper.getGipperStatus()
        status = self.gripper.getGipperStatus()
        return {
            'Ready'             : status[0],
            'Reset'             : status[1],
            'Is Moving'         : status[2],
            'Object Detected'   : status[3],
            'Fault Code'        : status[4],
            'Pos'               : status[5],
            'Requested Pos'     : status[6],
            'Current'           : status[7],
        }
    
    def initialize(self) -> bool:
        self.init = False
        import time
        print("Initializing gripper...")
        

        while not self.gripper.initialize():
            print("Trying to initialize gripper...")
            time.sleep(1)
        
        self.init = True
        return self.init
    
    def open(self) -> None:
        self.gripper.open_()
    
    def close(self) -> None:
        self.gripper.close_()

    def go_to(self, position, speed, force) -> None:
        self.gripper.goTo(position, speed, force)

if __name__ == "__main__":
    import os, sys
    sys.path.append(os.path.join(os.path.dirname(__file__)))
    # Esempio di utilizzo
    # gripper_control_socket = GripperControl('2F_85', commu_type='socket', robot_ip="192.168.0.102", port=63352)
    gripper_control_mdb = GripperControl('Hand_E', commu_type='mdb', id=1, comPort='/dev/ttyUSB0', baud_rate=115200, timeout=0.005)
    
    gripper_control_mdb.initialize()
    # gripper_control_mdb.go_to(100, 0.1, 10)