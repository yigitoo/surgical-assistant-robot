import rospy

class ListenCmd:
    def __init__(self, node_name = "listen_cmd"):
        self.node_name = node_name
        rospy.init_node(self.node_name)
