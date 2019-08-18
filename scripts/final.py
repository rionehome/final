#!/usr/bin/env python
# -*- coding: utf-8 -*-
import actionlib
import rospy
from std_msgs.msg import String, Bool
from sound_system.srv import StringService
from location.srv import RegisterLocation, RequestLocation
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from geometry_msgs.msg import Point, Quaternion
from module.rviz_marker import RvizMarker
import time
import difflib  # 類似度を計算するライブラリ


class RestaurantFinal:
    def __init__(self):
        rospy.init_node("final", anonymous=True)

        self.marker = RvizMarker()
        self.move_base_client = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
        
        self.call_ducker_pub = rospy.Publisher("/call_ducker/control", String, queue_size=10)

        rospy.Subscriber("/trigger/start", String, self.start_callback)
        rospy.Subscriber("/order_input", String, self.order_text_callback)
        rospy.Subscriber("/call_ducker/finish", Bool, self.call_ducker_callback)

    def start_callback(self, data):
        # type:(String) -> None
        """
        トリガーのcallback関数
        最初の発話
        :param data: 空文字
        :return:なし
        """
        self.print_function("start_callback")
        self.speak("Hello, everyone. I will start the demonstration now.")
        time.sleep(0.5)
        self.speak("If you want to order, please input the item from the tablet.")

    def order_text_callback(self, data):
        # type:(String) -> None
        """
        Webから入力文のsubscribeするcallback関数
        :param data: オーダーのテキスト
        :return: なし
        """
        self.print_function("order_text_callback")

        rospy.wait_for_service('/location/register_current_location', timeout=1)
        rospy.ServiceProxy('/location/register_current_location', RegisterLocation)("kitchen")

        
        web_text = data.data.lower()
        
        order_sentence_list = ["i want beans", "i want biscuits", "i want coca cola", "i want cookies",
                               "i want grape juice", "i want green tea", "i want olive", "i want onion soup", "i want pringles"]
        # 類似度計算
        score = 0
        order_sentence = ""
        for x in order_sentence_list:
            s = difflib.SequenceMatcher(None, x, web_text).ratio()
            if score < s:
                score = s
                order_sentence = x
        
        order = " ".join(order_sentence.split()[2:])  # 商品名

        self.speak("Order is {}.".format(order))
        self.speak("Please give me items.")
        
        time.sleep(5)
        
        self.speak("I will deliver {}.".format(order))
        
        # call_duckerにメッセージを送信
        self.call_ducker_pub.publish("start")
    
    def call_ducker_callback(self, msg):
        # type:(Bool) -> None
        """
        call_duckerのcallback関数
        call_ducekrの成功/失敗を判定
        :param: msg: Bool
        :return: なし
        """
        self.print_function("call_ducker_callback")
        if msg.data:
            # テーブルに着いた
            self.speak("Thank you for waiting. Here you are. So, I want you to take items.")

            time.sleep(5)

            rospy.wait_for_service("/location/request_location", timeout=1)
            response = rospy.ServiceProxy("/location/request_location", RequestLocation)("kitchen").location
            self.send_move_base(response)

        else:
            self.speak("Sorry, Please carry it in front of the customer.")
            print "お客さんの前に運んでください。"

    def send_move_base(self, point):
        goal = MoveBaseGoal()
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.pose.position = Point(point.x, point.y, point.z)
        goal.target_pose.pose.orientation = Quaternion(0, 0, 0, 1)
        self.marker.register(goal.target_pose.pose)
        self.move_base_client.wait_for_server()
        print "move_baseに送信"
        self.move_base_client.send_goal(goal)
        self.move_base_client.wait_for_result()
        return self.move_base_client.get_state()
    
    @staticmethod
    def speak(text):
        # type: (str) -> None
        """
        sound_system_ros に対して発話を要求する
        発話が終了するまで待機
        :param text: 発話内容
        :return: なし
        """
        rospy.wait_for_service("/sound_system/speak")
        rospy.ServiceProxy("/sound_system/speak", StringService)(text)
    
    @staticmethod
    def print_function(name):
        # type: (str) -> None
        """
        関数名を表示
        :return: なし
        """
        print("\n###########################################\n")
        print("     Function: {}".format(name))
        print("\n###########################################\n")


if __name__ == '__main__':
    RestaurantFinal()
    rospy.spin()
