#!/usr/bin/env python

# Copyright 2023 Philipp Schillinger, Team ViGIR, Christopher Newport University
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Philipp Schillinger, Team ViGIR, Christopher Newport University nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


from flexbe_core import EventState, Logger

from flexbe_core.proxy import ProxySubscriberCached


class SubscriberState(EventState):
    """
    Gets the latest message on the given topic and stores it to userdata.

    -- topic 		string		The topic on which should be listened.
    -- blocking 	bool 		Blocks until a message is received.
    -- clear 		bool 		Drops last message on this topic on enter
                                in order to only handle message received since this state is active.

    #> message		object		Latest message on the given topic of the respective type.

    <= received 				Message has been received and stored in userdata or state is not blocking.
    <= unavailable 				The topic is not available when this state becomes actives.
    """

    def __init__(self, topic, msg_type="", blocking=True, clear=False):
        super(SubscriberState, self).__init__(outcomes=['received', 'unavailable'],
                                              output_keys=['message'])
        self._topic = topic
        self._msg_type = msg_type
        self._blocking = blocking
        self._clear = clear
        self._connected = False
        self._sub = None
        if not self._connect():
            Logger.logwarn('Topic %s for state %s not yet available.\n'
                           'Will try again when entering the state...' % (self._topic, self.name))

    def on_stop(self):
        if self._connected:
            ProxySubscriberCached.unsubscribe_topic(self._topic)
            self._connected = False

    def execute(self, userdata):
        if not self._connected:
            userdata.message = None
            return 'unavailable'

        if self._sub.has_msg(self._topic) or not self._blocking:
            userdata.message = self._sub.get_last_msg(self._topic)
            self._sub.remove_last_msg(self._topic)
            return 'received'

        return None

    def on_enter(self, userdata):
        if not self._connected:
            if self._connect():
                Logger.loginfo('Successfully subscribed to previously unavailable topic %s' % self._topic)
            else:
                Logger.logwarn('Topic %s still not available, giving up.' % self._topic)

        if self._connected and self._clear and self._sub.has_msg(self._topic):
            self._sub.remove_last_msg(self._topic)

    def _connect(self):
        try:
            self._sub = ProxySubscriberCached({self._topic: self._msg_type}, inst_id=id(self))
            self._connected = True
            return True
        except Exception:  # pylint: disable=W0703
            return False
