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


"""Main function to start up the FlexBE mirror of onboard statemachine."""
from datetime import datetime
import rclpy

from flexbe_core.proxy import shutdown_proxies
from flexbe_mirror.flexbe_mirror import FlexbeMirror


def main(args=None):
    """Run main function to start up the FlexBE mirror of onboard statemachine."""
    rclpy.init(args=args,
               signal_handler_options=rclpy.signals.SignalHandlerOptions.NO)  # We will handle shutdown

    mirror = FlexbeMirror()

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(mirror)

    mirror.get_logger().info("Begin behavior mirror processing ...")

    # Wait for ctrl-c to stop the application
    try:
        executor.spin()
    except KeyboardInterrupt:
        print(f"Keyboard interrupt at {datetime.now()} ! Shut the behavior mirror down!", flush=True)
    except Exception as exc:
        print(f"Exception in mirror executor! {type(exc)}\n  {exc}", flush=True)
        import traceback
        print(f"{traceback.format_exc().replace('%', '%%')}", flush=True)

    try:
        print(f"Request behavior mirror shutdown at {datetime.now()} ...", flush=True)
        if mirror.shutdown_mirror():
            print(f"Mirror shutdown at {datetime.now()} ...", flush=True)
        else:
            # Last call for clean up of any stray communications and try again
            for _ in range(100):
                executor.spin_once(timeout_sec=0.001)  # allow behavior to cleanup after itself
            if mirror.shutdown_mirror():
                print(f"Mirror shutdown at {datetime.now()} ...", flush=True)

    except Exception as exc:
        print(f"Exception in behavior mirror node shutdown! {type(exc)}\n  {exc}", flush=True)
        import traceback
        print(f"{traceback.format_exc().replace('%', '%%')}", flush=True)

    try:
        print(f"Shutdown proxies requested  at {datetime.now()} ...", flush=True)
        shutdown_proxies()

        # Last call for clean up of any stray communications
        for _ in range(100):
            executor.spin_once(timeout_sec=0.001)  # allow behavior to cleanup after itself

        print(f"Mirror proxy  shutdown completed  at {datetime.now()} ...", flush=True)
        mirror.destroy_node()
    except Exception as exc:
        print(f"Exception in behavior mirror node shutdown! {type(exc)}\n  {exc}", flush=True)
        import traceback
        print(f"{traceback.format_exc().replace('%', '%%')}", flush=True)

    print(f"Done with behavior mirror at {datetime.now()}!", flush=True)
    try:
        rclpy.try_shutdown()
    except Exception as exc:  # pylint: disable=W0703
        print(f"Exception from rclpy.try_shutdown for behavior mirror: {type(exc)}\n{exc}", flush=True)
        print(f"{traceback.format_exc().replace('%', '%%')}", flush=True)


if __name__ == '__main__':
    main()
