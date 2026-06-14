import zmq
import threading
import json
import time
import pickle

class ZMQPubSub:
    def __init__(self, port=5555):
        self.context = zmq.Context()
        self.port = port

    def create_publisher(self):
        return ZMQPublisher(self.context, self.port)

    def create_subscriber(self):
        return ZMQSubscriber(self.context, self.port)

class ZMQPublisher:
    def __init__(self, context, port=5555):
        self.socket = context.socket(zmq.PUB)
        self.socket.bind(f"tcp://*:{port}")
        time.sleep(0.2)
        
    def publish(self, topic, message):
        self.socket.send_string(f"{topic} {json.dumps(message)}")
        
    def publish_array(self, topic, message):
        self.socket.send_string(topic, zmq.SNDMORE)
        self.socket.send(pickle.dumps(message, protocol=pickle.HIGHEST_PROTOCOL))

class ZMQSubscriber:
    def __init__(self, context, port=5555):
        self.socket = context.socket(zmq.SUB)
        self.socket.connect(f"tcp://localhost:{port}")
        self.callbacks = {}

    def subscribe(self, topic, callback):
        self.socket.setsockopt_string(zmq.SUBSCRIBE, topic)
        self.callbacks[topic] = callback

    def start(self):
        threading.Thread(target=self._listen, daemon=True).start()

    def _listen(self):
        while True:
            try:
                topic = self.socket.recv_string()
                data = self.socket.recv()
                if topic in self.callbacks:
                    message = pickle.loads(data)
                    self.callbacks[topic](message)
            except Exception as e:
                print(f"[ZMQ] Subscriber error: {e}")