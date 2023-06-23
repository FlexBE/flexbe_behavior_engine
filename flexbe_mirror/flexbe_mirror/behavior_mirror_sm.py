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
import multiprocessing
from threading import Event
import rclpy

from flexbe_core.proxy import shutdown_proxies
from flexbe_mirror.flexbe_mirror import FlexbeMirror


def main(args=None):
    """Run main function to start up the FlexBE mirror of onboard statemachine."""
    rclpy.init(args=args)

    mirror = FlexbeMirror()

    # Use at least 2 threads, but don't hog the whole machine otherwise
    num_threads = max(2, multiprocessing.cpu_count() - 2)
    mirror.get_logger().info(f"Using {num_threads} threads in mirror executor ...")
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=num_threads)
    executor.add_node(mirror)

    mirror.get_logger().info("Begin processing ...")

    # Wait for ctrl-c to stop the application
    try:
        executor.spin()
    except KeyboardInterrupt:
        print("Keyboard interrupt! Shut the behavior mirror down!")
    except Exception as exc:
        print(f"Exception in mirror executor! {type(exc)}\n  {exc}")
        import traceback
        print(f"{traceback.format_exc().replace('%', '%%')}")

    try:
        print("Behavior mirror shutdown ...")
        Event().wait(0.25)  # Allow some time for threads to stop
        shutdown_proxies()
        mirror.destroy_node()
    except Exception as exc:
        print(f"Exception in behavior mirror node shutdown! {type(exc)}\n  {exc}")
        import traceback
        print(f"{traceback.format_exc().replace('%', '%%')}")

    print("Done with behavior mirror!")
    try:
        rclpy.try_shutdown()
    except Exception as exc:  # pylint: disable=W0703
        print(f"Exception from rclpy.try_shutdown for behavior mirror: {type(exc)}\n{exc}")
        print(f"{traceback.format_exc().replace('%', '%%')}")


if __name__ == '__main__':
    main()
