#!/usr/bin/python
import rospy
import apiai
import uuid
import json

from std_msgs.msg import String

CLIENT_ACCESS_TOKEN = '5e8f197999b74c32a32219df882e88cf'

class DialogflowHandler:

	def __init__(self):

		self.speech = ''
		self.ai = apiai.ApiAI(CLIENT_ACCESS_TOKEN)
		self.session_id = uuid.uuid4().hex
		
		self.google_pub = rospy.Publisher('/google_speech_output', String, queue_size=10)
		rospy.Subscriber('/speech_text_input', String, self.get_response)
		rospy.spin()

	def get_response(self, data):
		request = self.ai.text_request()
		request.lang = 'en'
		request.session_id = self.session_id
		request.query = data.data
		# Send request to api.ai
		response_data = request.getresponse()
		response = json.loads(response_data.read())
		
		self.process_response(response)

	def process_response(self, resp):
		self.speech = resp['result']['fulfillment']['speech']
		rospy.loginfo('speech: %s'%(self.speech))
		# Publish speech output
		self.google_pub.publish(self.speech)



if __name__ == '__main__':
    rospy.init_node('google_intent_handler', anonymous=False)
    try:
    	g = DialogflowHandler();
    except rospy.ROSInterruptException: pass
