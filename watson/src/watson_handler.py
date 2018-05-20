#!/usr/bin/python
import json
import rospy
import watson_developer_cloud

from std_msgs.msg import String

USERNAME = 'd4800555-9a61-4bce-8a30-17d7aa12d2da'
PASSWORD = 'Lo3zZbIDE3Mo'
VERSION = '2018-04-12'
# WORKSPACE_ID = '6cdc9c42-a579-4f2b-a90c-3b27c1198113'
WORKSPACE_ID = '3ddfa9f7-5f07-484a-9379-c61635ca8fb6'

class WatsonHandler:

	def __init__(self):
		self.speech = ''
		self.assistant = watson_developer_cloud.AssistantV1(
			username=USERNAME,
			password=PASSWORD,
			version=VERSION
		)
		self.watson_pub = rospy.Publisher('/watson_speech_output', String, queue_size=10)
		rospy.Subscriber('/speech_text_input', String, self.get_response)
		rospy.spin()

	def get_response(self, data):
		response = self.assistant.message(
			workspace_id = WORKSPACE_ID,
			input={
				'text': data.data
			}
		)
		self.process_response(response)

	def process_response(self, resp):
		print(json.dumps(resp, indent=2))
		
		self.speech = resp['output']['text']['speech']
		rospy.loginfo('speech: %s'%(self.speech))
		# Publish speech output
		self.watson_pub.publish(self.speech)



if __name__ == '__main__':
	rospy.init_node('watson_intent_handler', anonymous=False)
	try:
		w = WatsonHandler();
	except rospy.ROSInterruptException: pass