########################################################################
#
# Copyright (c) 2017, STEREOLABS.
#
# All rights reserved.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
########################################################################

"""
    Position sample shows the position of the ZED camera in a OpenGL window.
"""

import pyzed.sl as sl
import threading



def main():

    init = sl.InitParameters(camera_resolution=sl.RESOLUTION.RESOLUTION_HD720,
                                 depth_mode=sl.DEPTH_MODE.DEPTH_MODE_PERFORMANCE,
                                 coordinate_units=sl.UNIT.UNIT_METER,
                                 coordinate_system=sl.COORDINATE_SYSTEM.COORDINATE_SYSTEM_RIGHT_HANDED_Y_UP,
                                 sdk_verbose=True)
    cam = sl.Camera()
    status = cam.open(init)
    if status != sl.ERROR_CODE.SUCCESS:
        print(repr(status))
        exit()

    transform = sl.Transform()
    tracking_params = sl.TrackingParameters(transform)
    cam.enable_tracking(tracking_params)

    runtime = sl.RuntimeParameters()
    camera_pose = sl.Pose()


    py_translation = sl.Translation()

    start_zed(cam, runtime, camera_pose, py_translation)


def start_zed(cam, runtime, camera_pose, py_translation):
    zed_callback = threading.Thread(target=run, args=(cam, runtime, camera_pose, py_translation))
    zed_callback.start()


def run(cam, runtime, camera_pose, py_translation):
    while True:
        if cam.grab(runtime) == sl.ERROR_CODE.SUCCESS:
            tracking_state = cam.get_position(camera_pose)
            text_translation = ""
            text_rotation = ""
            if tracking_state == sl.TRACKING_STATE.TRACKING_STATE_OK:
                rotation = camera_pose.get_rotation_vector()
                rx = round(rotation[0], 2)
                ry = round(rotation[1], 2)
                rz = round(rotation[2], 2)

                translation = camera_pose.get_translation(py_translation)
                tx = round(translation.get()[0], 2)
                ty = round(translation.get()[1], 2)
                tz = round(translation.get()[2], 2)

                pose_data = camera_pose.pose_data(sl.Transform())

                print(tx, ty, tz)

        else:
            sl.c_sleep_ms(1)


if __name__ == "__main__":
    main()