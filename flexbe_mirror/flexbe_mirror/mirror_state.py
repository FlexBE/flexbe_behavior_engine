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


"""Simplified state for use with FlexBE UI State machine mirror."""

from std_msgs.msg import String, UInt8

from flexbe_core import EventState
from flexbe_core.proxy import ProxyPublisher, ProxySubscriberCached


class MirrorState(EventState):
    """
    Simplified state for use with FlexBE UI State machine mirror.

    This state will display its possible outcomes as buttons in the GUI
    and is designed in a way to be created dynamically.
    """

    def __init__(self, target_name, target_path, given_outcomes, outcome_autonomy):
        # pylint: disable=unused-argument
        super().__init__(outcomes=given_outcomes)
        self.set_rate(100)
        self._target_name = target_name
        self._target_path = target_path

        self._outcome_topic = 'flexbe/mirror/outcome'

        self._pub = ProxyPublisher()
        self._sub = ProxySubscriberCached({self._outcome_topic: UInt8}, inst_id=id(self))

    def execute(self, userdata):
        if self._sub.has_buffered(self._outcome_topic):
            msg = self._sub.get_from_buffer(self._outcome_topic)
            if msg.data < len(self.outcomes):
                return self.outcomes[msg.data]
        return None

    def on_enter(self, userdata):
        self._pub.publish('flexbe/behavior_update',
                          String(data="/" + "/".join(self._target_path.split("/")[1:])))
