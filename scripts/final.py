#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String, Bool
from sound_system.srv import StringService
import time
import difflib  # 類似度を計算するライブラリ


class RestaurantFinal:
    def __init__(self):
        rospy.init_node("final_finish", anonymous=True)
        
        self.call_ducker_pub = rospy.Publisher("/call_ducker/control", String, queue_size=10)
        
        rospy.Subscriber("/order_input", String, self.order_text_callback)
        rospy.Subscriber("/call_ducker/finish", Bool, self.call_ducker_callback)
    
    def order_text_callback(self, data):
        # type:(String) -> None
        """
        Webから入力文のsubscribeするcallback関数
        :param data: オーダーのテキスト
        :return: なし
        """
        self.print_function("order_text_callback")
        
        web_text = data.data.lower()
        
        order_sentence_list = ["i want beans", "i want biscuits", "i want coca cola", "i want cookies",
                               "i want grape juice"]
        # 類似度計算
        score = 0
        order_sentence = ""
        for x in order_sentence_list:
            s = difflib.SequenceMatcher(None, x, web_text).ratio()
            if score < s:
                score = s
                order_sentence = x
        
        order = " ".join(order_sentence.split()[2:])  # 商品名
        
        # キッチンに着いた
        self.speak("Order is {}.".format(order))
        self.speak("Please give me items.")
        
        time.sleep(5)
        
        self.speak("I will deliver {}. Please raise your hand.".format(order))
        
        # call_duckerにメッセージを送信
        self.call_ducker_pub.publish("start")
    
    def call_ducker_callback(self, msg):
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
        else:
            self.speak("Sorry, Please carry it in front of the customer.")
            print "お客さんの前に運んでください。"
    
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
        ノード名を表示
        :return: なし
        """
        print("\n###########################################\n")
        print("     Function: {}".format(name))
        print("\n###########################################\n")


if __name__ == '__main__':
    RestaurantFinal()
    rospy.spin()
