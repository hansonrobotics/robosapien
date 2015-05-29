#!/usr/bin/env python

#from chatbot.msg import ChatMessage
#from OpenSSL import SSL
from flask import Flask
from flask import render_template
#from flask_sslify import SSLify
import rospy

#context = SSL.Context(SSL.SSLv23_METHOD)
#context.use_privatekey_file('yourserver.key')
#context.use_certificate_file('yourserver.crt')
#context = SSL.SSLContext(SSL.PROTOCOL_TLSv1_2)
#context.load_cert_chain('yourserver.crt', 'yourserver.key')
app = Flask(__name__)
app.config['WS_HOST'] = 'localhost'
#ss=SSLify(app)
@app.route("/")
def home():
  return render_template('index.html', ws_host=app.config['WS_HOST'])
#index.html
#@app.route("/speech")
#def speech():
#  return render_template('speech.html')


class ChatbotWebSpeechRecognizer:
  def __init__(self):
    rospy.init_node('chatbot_web_listener')

  def start(self):
    context = ('/home/mandeep/self_ssl/server.crt', '/home/mandeep/self_ssl/server.key')
    #app.run(host='0.0.0.0', port=5000, ssl_context=context, threaded=True, debug=True)
    app.run(host='0.0.0.0', port=5000, ssl_context=context)
    #app.run(host='0.0.0.0')

def main():
  recognizer = ChatbotWebSpeechRecognizer()
  recognizer.start() 

if __name__ == '__main__':
  main()
