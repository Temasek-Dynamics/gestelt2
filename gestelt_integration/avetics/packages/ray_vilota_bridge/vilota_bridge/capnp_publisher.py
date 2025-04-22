from ecal.core.publisher import MessagePublisher

class CapnpPublisher(MessagePublisher):
  """Spezialized publisher that sends out raw bytes
  """
  def __init__(self, name, type):
    self.topic_type = "capnp:" + type
    topic_desc = b""
    super(CapnpPublisher, self).__init__(name, self.topic_type, topic_desc)

  def send(self, msg, time=-1):
    return self.c_publisher.send(msg, time)