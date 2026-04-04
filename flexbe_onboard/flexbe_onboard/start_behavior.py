# Copyright 2024 Philipp Schillinger, Team ViGIR, Christopher Newport University
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
#    * Neither the name of the Philipp Schillinger, Team ViGIR,
#      Christopher Newport University nor the names of its
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


"""Script for starting the onboard behavior engine."""

from datetime import datetime

from flexbe_core.proxy import shutdown_proxies

from flexbe_onboard.flexbe_onboard import FlexbeOnboard

import rclpy
from rclpy._rclpy_pybind11 import InvalidHandle
from rclpy.executors import ExternalShutdownException


def _spin_cleanup(executor, iterations, context, timeout_sec=0.001):
    """Drain executor work until cleanup completes or ROS starts tearing handles down."""
    for _ in range(iterations):
        try:
            executor.spin_once(timeout_sec=timeout_sec)
        except (ExternalShutdownException, InvalidHandle) as exc:
            print(f'Executor cleanup interrupted during {context} at {datetime.now()} - '
                  f'{type(exc).__name__}: {exc}', flush=True)
            return False
    return True


def main(args=None):
    """Script for starting the onboard behavior engine."""
    rclpy.init(args=args,
               signal_handler_options=rclpy.signals.SignalHandlerOptions.NO)  # We will handle shutdown

    onboard = FlexbeOnboard()

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(onboard)
    onboard.executor = executor

    onboard.get_logger().info('Begin processing onboard behavior engine ...')

    # Wait for ctrl-c to stop the application
    try:
        executor.spin()
    except KeyboardInterrupt:
        print(f'Keyboard interrupt request  at {datetime.now()} - ! Shut the onboard behavior executive down!', flush=True)
    except (ExternalShutdownException, InvalidHandle) as exc:
        print(f'Executor stopped during shutdown at {datetime.now()} - '
              f'{type(exc).__name__}: {exc}', flush=True)
    except Exception as exc:
        print(f'Exception in executor       at {datetime.now()} - ! {type(exc)}\n  {exc}', flush=True)
        import traceback
        print(f"{traceback.format_exc().replace('%', '%%')}", flush=True)

    # Attempt to do a clean shutdown of any active behavior then the behavior engine itself
    try:
        print(f'Request onboard behavior shutdown   at {datetime.now()} ...', flush=True)
        if onboard.behavior_shutdown():
            for i in range(5):
                if not _spin_cleanup(executor, 100 * i, 'behavior shutdown'):
                    break
                if onboard.verify_no_active_behaviors(timeout=0.1):
                    break
                else:
                    print(f'    Active behavior still running onboard={onboard._running} at {datetime.now()}!', flush=True)
        else:
            print(f'    All onboard behaviors are stopped at {datetime.now()}!', flush=True)

        # Last call for clean up of any stray communications
        _spin_cleanup(executor, 50, 'post-behavior cleanup')

        print(f'{datetime.now()} - onboard shutdown requested ...', flush=True)
        onboard.onboard_shutdown()
        _spin_cleanup(executor, 30, 'onboard shutdown')

    except Exception as exc:
        print(f"{datetime.now()} - Exception in onboard shutdown! '{type(exc)}'\n  {exc}", flush=True)
        import traceback
        print(f"{traceback.format_exc().replace('%', '%%')}", flush=True)

    try:
        print(f'Shutdown proxies requested  at {datetime.now()} ...', flush=True)
        shutdown_proxies()

        print(f'Proxy  shutdown  completed  at {datetime.now()} ...', flush=True)
        executor.remove_node(onboard)
        onboard.destroy_node()
        executor.shutdown()

        print(f'Node destruction  completed at {datetime.now()} ...', flush=True)
    except Exception as exc:
        print(f"{datetime.now()} - Exception in onboard proxy and node shutdown! '{type(exc)}'\n  {exc}", flush=True)
        import traceback
        print(f"{traceback.format_exc().replace('%', '%%')}", flush=True)

    print(f'Done with behavior executive at {datetime.now()}!', flush=True)
    try:
        rclpy.try_shutdown()
    except Exception as exc:  # pylint: disable=W0703
        import traceback
        print(f"{datetime.now()} - Exception from rclpy.shutdown for start onboard behavior: '{type(exc)}'\n{exc}", flush=True)
        print(f"{traceback.format_exc().replace('%', '%%')}", flush=True)


if __name__ == '__main__':
    main()
