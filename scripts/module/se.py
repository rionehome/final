import rospkg
import subprocess


class SE:
    def __init__(self):
        self.se_path = "{}/etc/SE/".format(rospkg.RosPack().get_path('final'))
        self.DISCOVERY = self.se_path + "discovery.wav"
        self.WARNING = self.se_path + "warning.wav"
    
    @staticmethod
    def play(se):
        # type: (str) -> None
        subprocess.call(["aplay", se], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
