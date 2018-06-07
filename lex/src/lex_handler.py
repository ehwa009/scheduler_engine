#!/usr/bin/python
import rospy
from std_msgs.msg import String
import json
import uuid
import boto3

class LexHandler: 
    
    def __init__(self):
        
        self.speech = ''
        self.session_id = uuid.uuid4().hex
        self.session_attributes = {}
        
        self.client = boto3.client('lex-runtime')
        self.comprehend = boto3.client(service_name='comprehend', region_name='us-east-1')
        
        self.lex_pub = rospy.Publisher('/lex_speech_output', String, queue_size=10)
        rospy.Subscriber('/speech_text_input', String, self.get_respones)

    def get_respones(self, data):
        response = self.client.post_content(
            botName = 'Receptionist',
            botAlias = 'reception',
            userId = self.session_id,
            contentType = 'text/plain; charset=utf-8',
            sessionAttributes = self.session_attributes,
            accept = 'text/plain; charset=utf-8',
            inputStream = data.data
        )
        self.process_response(response)

        print(json.dumps(self.comprehend.detect_sentiment(Text=data.data, LanguageCode='en'), sort_keys=True, indent=4))
        
    def process_response(self, resp):
        self.speech = resp.get('message')
        rospy.loginfo('Speech: %s'%(self.speech))
        # Publish intent data
        self.lex_pub.publish(self.speech)



if __name__ == '__main__':
    rospy.init_node('lex_intent_handler', anonymous=False)
    l = LexHandler()
    rospy.spin()

