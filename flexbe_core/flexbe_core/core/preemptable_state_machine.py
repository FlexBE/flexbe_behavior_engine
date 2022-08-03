#!/usr/bin/env python
import threading
import zlib
from flexbe_core.core.preemptable_state import PreemptableState
from flexbe_core.logger import Logger
from flexbe_core.proxy import ProxySubscriberCached

from std_msgs.msg import Empty
from flexbe_msgs.msg import  BehaviorSync

from flexbe_core.core.lockable_state_machine import LockableStateMachine


class PreemptableStateMachine(LockableStateMachine):
    """
    A state machine that can be preempted.
    If preempted, the state machine will return the outcome preempted.
    """

    _preempted_name = 'preempted'

    def __init__(self, *args, **kwargs):
        super(PreemptableStateMachine, self).__init__(*args, **kwargs)
        # always listen to preempt so that the behavior can be stopped even if unsupervised
        self._preempt_topic = 'flexbe/command/preempt'
        self._sub = ProxySubscriberCached({self._preempt_topic: Empty}, id=id(self))
        self._sub.set_callback(self._preempt_topic, self._preempt_cb, id=id(self))
        self._status_lock = threading.Lock()
        self._last_deep_state_name = None
        self._last_deep_state_path = None

    def _preempt_cb(self, msg):
        if not self._is_controlled:
            PreemptableState.preempt = True

    @staticmethod
    def add(label, state, transitions=None, remapping=None):
        transitions[PreemptableState._preempted_name] = PreemptableStateMachine._preempted_name
        LockableStateMachine.add(label, state, transitions, remapping)

    @property
    def _valid_targets(self):
        return super(PreemptableStateMachine, self)._valid_targets + [PreemptableStateMachine._preempted_name]

    def spin(self, userdata=None):
        outcome = None
        while True:
            outcome = self.execute(userdata)

            # Store the information for safely passing to heartbeat thread
            deep_state = self.get_deep_state()
            if deep_state:
                if deep_state.name != self._last_deep_state_name:
                    with self._status_lock:
                        self._last_deep_state_path = str(deep_state.path)
                        self._last_deep_state_name = str(deep_state.name)
            else:
                if self._last_deep_state_name != None:
                    with self._status_lock:
                        self._last_deep_state_path = None
                        self._last_deep_state_name = None

            if self._inner_sync_request:
                # Top-level state machine with sync request
                self.process_sync_request()

            if outcome is not None:
                break

            self.sleep()
        return outcome

    def get_latest_status(self):
        """
        Returns the latest execution information as a BehaviorSync message
        This version is typically called by the OCS mirror, so we do
        some extra cleanup.
        """
        with self._status_lock:
            path = self._last_deep_state_path

        msg = BehaviorSync()
        msg.behavior_id = -1
        if path is not None:
            path_clean = path.replace("_mirror", "")  # Ignore mirror decoration for comparison with onboard
            msg.current_state_checksum = zlib.adler32(path_clean.encode()) & 0x7fffffff
        else:
            msg.current_state_checksum = -1
        return msg


    def process_sync_request(self):
        Logger.localinfo("Ignoring PreemptableState process_sync_request")
