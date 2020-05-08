from fabrik_full_body.singleton import Singleton
class OutputWriter(metaclass=Singleton):
    def __init__(self, base_address="./outputs/", angles_file_address = "angles.txt"):
        self.base_address = base_address
        self.angles_file_address = base_address + angles_file_address
    
    def angle_writer(self):
        return open(self.angles_file_address, "w")
