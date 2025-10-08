#!/usr/bin/env python3

# Copyright 2024 Christopher Newport University
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
#    * Neither the name of the Christopher Newport University nor the names of its
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

from flexbe_msgs.action import BehaviorExecution
from flexbe_msgs.msg import BEStatus, BehaviorLog, BehaviorRequest, BehaviorSelection, BehaviorSync
from flexbe_msgs.msg import CommandFeedback, ContainerStructure
from flexbe_msgs.msg import OutcomeRequest
from flexbe_msgs.msg import StateMapMsg

from std_msgs.msg import Bool, Empty, Int32, String, UInt32, UInt8


class Topics:
    """Standardize handling of FlexBE topics."""

    _BEHAVIOR_LOGGING_TOPIC = 'flexbe/log'  # Logger topic
    _BEHAVIOR_UPDATE_TOPIC = 'flexbe/behavior_update'
    _CMD_AUTONOMY_TOPIC = 'flexbe/command/autonomy'  # OCS request autonomy level
    _CMD_ATTACH_TOPIC = 'flexbe/command/attach'  # OCS request attach UI and Mirror to running behavior
    _CMD_FEEDBACK_TOPIC = 'flexbe/command_feedback'  # Feedback to OCS of command outcome
    _CMD_LOCK_TOPIC = 'flexbe/command/lock'  # OCS request to lock state
    _CMD_PAUSE_TOPIC = 'flexbe/command/pause'  # OCS request to pause execution
    _CMD_PREEMPT_TOPIC = 'flexbe/command/preempt'  # OCS request behavior preempt
    _CMD_REPEAT_TOPIC = 'flexbe/command/repeat'  # OCS request to repeat execution of a single EventState implementation
    _CMD_SYNC_TOPIC = 'flexbe/command/sync'  # OCS request synchronization
    _CMD_TRANSITION_TOPIC = 'flexbe/command/transition'  # OCS request transition
    _CMD_UNLOCK_TOPIC = 'flexbe/command/unlock'  # OCS request to unlock state
    _DEBUG_TOPIC = 'flexbe/debug/current_state'  # Used for logging state entry/exit
    _LAUNCHER_HEARTBEAT_TOPIC = 'flexbe/launcher/heartbeat'  # Clock seconds active
    _MIRROR_HEARTBEAT_TOPIC = 'flexbe/mirror/heartbeat'  # Clock seconds active
    _MIRROR_PREEMPT_TOPIC = 'flexbe/mirror/preempt'  # Preempt
    _MIRROR_STRUCTURE_TOPIC = 'flexbe/mirror/structure'  # Pass behavior structure back to mirror
    _MIRROR_SYNC_TOPIC = 'flexbe/mirror/sync'  # Trigger mirror to re-synchronize with onboard
    _ONBOARD_HEARTBEAT_TOPIC = 'flexbe/heartbeat'  # Onboard behavior executive is alive
    _ONBOARD_STATUS_TOPIC = 'flexbe/status'  # Onboard behavior engine status
    _OUTCOME_REQUEST_TOPIC = 'flexbe/outcome_request'  # OCS request outcome
    _OUTCOME_TOPIC = 'flexbe/mirror/outcome'  # State outcomes used in mirror to track status
    _REQUEST_BEHAVIOR_TOPIC = 'flexbe/request_behavior'
    _REQUEST_STRUCTURE_TOPIC = 'flexbe/request_mirror_structure'  # Request state machine structure from onboard
    _START_BEHAVIOR_TOPIC = 'flexbe/start_behavior'  # OCS or launcher command to start behavior
    _STATE_LOGGER_TOPIC = 'flexbe/state_logger'
    _STATE_MAP_TOPIC = 'flexbe/state_map'  # Mapping of state id hash codes to state path from onboard
    _STATE_MAP_OCS_TOPIC = 'flexbe/mirror/state_map'  # Mapping of state id hash codes to state path from mirror (should be same)
    _UI_VERSION_TOPIC = 'flexbe/ui_version'  # OCS Version topic

    # Action interfaces
    _EXECUTE_BEHAVIOR_ACTION = 'flexbe/execute_behavior'

    _topic_types = {_BEHAVIOR_LOGGING_TOPIC: BehaviorLog,
                    _BEHAVIOR_UPDATE_TOPIC: Int32,
                    _CMD_ATTACH_TOPIC: UInt8,
                    _CMD_AUTONOMY_TOPIC: UInt8,
                    _CMD_FEEDBACK_TOPIC: CommandFeedback,
                    _CMD_LOCK_TOPIC: Int32,
                    _CMD_PAUSE_TOPIC: Bool,
                    _CMD_PREEMPT_TOPIC: Empty,
                    _CMD_REPEAT_TOPIC: Empty,
                    _CMD_SYNC_TOPIC: Empty,
                    _CMD_TRANSITION_TOPIC: OutcomeRequest,
                    _CMD_UNLOCK_TOPIC: Int32,
                    _DEBUG_TOPIC: String,
                    _EXECUTE_BEHAVIOR_ACTION: BehaviorExecution,
                    _LAUNCHER_HEARTBEAT_TOPIC: UInt32,
                    _MIRROR_HEARTBEAT_TOPIC: UInt32,
                    _MIRROR_PREEMPT_TOPIC: Empty,
                    _MIRROR_STRUCTURE_TOPIC: ContainerStructure,
                    _MIRROR_SYNC_TOPIC: BehaviorSync,
                    _ONBOARD_HEARTBEAT_TOPIC: BehaviorSync,
                    _ONBOARD_STATUS_TOPIC: BEStatus,
                    _OUTCOME_REQUEST_TOPIC: OutcomeRequest,
                    _OUTCOME_TOPIC: UInt32,
                    _REQUEST_BEHAVIOR_TOPIC: BehaviorRequest,
                    _REQUEST_STRUCTURE_TOPIC: UInt32,
                    _START_BEHAVIOR_TOPIC: BehaviorSelection,
                    _STATE_MAP_TOPIC: StateMapMsg,
                    _STATE_MAP_OCS_TOPIC: StateMapMsg,
                    _UI_VERSION_TOPIC: String
                    }

    @staticmethod
    def get_type(topic):
        """Return topic msg type given topic name."""
        return Topics._topic_types[topic]
