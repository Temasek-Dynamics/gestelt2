from ecal.core.subscriber import MessageSubscriber

class CapnpSubscriber(MessageSubscriber):
  """Specialized publisher subscribes to raw bytes
  """
  def __init__(self, type, name, typeclass=None):
    self.topic_type = "capnp:" + type
    super(CapnpSubscriber, self).__init__(name, self.topic_type)
    self.callback = None
    self.typeclass = typeclass

  def receive(self, timeout=0):
    """ receive subscriber content with timeout

    :param timeout: receive timeout in ms

    """
    ret, msg, time = self.c_subscriber.receive(timeout)
    return ret, msg, time

  def set_callback(self, callback):
    """ set callback function for incoming messages

    :param callback: python callback function (f(topic_name, msg, time))

    """
    self.callback = callback
    self.c_subscriber.set_callback(self._on_receive)

  def rem_callback(self, callback):
    """ remove callback function for incoming messages

    :param callback: python callback function (f(topic_type, topic_name, msg, time))

    """
    self.c_subscriber.rem_callback(self._on_receive)
    self.callback = None

  def _on_receive(self, topic_name, msg, time):
    if self.typeclass is None:
      self.callback(self.topic_type, topic_name, msg, time)
    else:
      with self.typeclass.from_bytes(msg) as msg:
        self.callback(self.topic_type, topic_name, msg, time)
