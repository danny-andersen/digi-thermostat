from time import sleep
from fractions import Fraction
from datetime import datetime
import subprocess
from multiprocessing import Process
import os
from picamera import PiCamera

WEB_CAM = True
DATE_FORMAT: str = "%Y%m%dT%H%M%S"
STATUS_FILE = "/home/danny/digi-thermostat/masterstation/status.txt"
MAX_LAST_UPDATE = 5 * 60
TAKE_PHOTO_FILE: str = "/home/danny/digi-thermostat/monitor_home/take-photo.txt"
DEVICES_FILE = "/home/danny/digi-thermostat/monitor_home/lan_devices.txt"
RECORD_TIME = 15  # Number of seconds to record each video segment


class Camera:
    _webcam_enabled: bool
    _camera: PiCamera
    photoVideoDir: str = "/home/danny/digi-thermostat/monitor_home/motion_images/"
    photoFilenamePi: str = "-piphoto.jpeg"
    photoFilenameWeb: str = "-webphoto.jpeg"
    videoFilename: str = "-pioutput"

    # TODO move these to a seperate config txt file
    ffmegStr = "ffmpeg -f v4l2 -video_size 1280x720 -i /dev/video0 -r 3.0 -frames 1 -y -loglevel error"
    mp4ConvStr = "MP4Box -add"

    def __init__(self, web_cam_on: bool, camera: PiCamera = None):
        self._webcam_enabled = web_cam_on
        self._camera = camera

    def _getCamera(self):
        if not self._camera:
            self._camera = PiCamera()
            self._camera.framerate = Fraction(25, 1)
            self._camera.iso = 800
        return self._camera

    def _closeCamera(self):
        if self._camera:
            self._camera.close()
            self._camera = None

    def takePiCamPhoto(self, dateStr: str):
        # Take photo with picam
        camera = self._getCamera()
        camera.resolution = (1024, 768)
        camera.capture(f"{self.photoVideoDir}{dateStr}{self.photoFilenamePi}")
        self._closeCamera()

    def takeWebCamPhoto(self, dateStr: str):
        # Take webcam photo
        ffmegCmd: list = self.ffmegStr.split()
        ffmegCmd.append(f"{self.photoVideoDir}{dateStr}{self.photoFilenameWeb}")
        subprocess.run(args=ffmegCmd)

    def takePhoto(self, dateStr: str):
        # Take photos in parallel using Process and then wait until they are done
        cameraProcess = Process(target=self.takePiCamPhoto)
        cameraProcess.start()
        if self._webcam_enabled:
            webCamProc = Process(target=self.takeWebCamPhoto)
            webCamProc.start()
            webCamProc.join()
        cameraProcess.join()

    def takeVideo(self, dateStr: str):
        # TODO: Work out how to take webcam video
        # TODO: Take videos in parallel using Process
        outfile: str = f"{self.photoVideoDir}{dateStr}{self.videoFilename}"
        camera = self._getCamera()
        camera.start_recording(
            output=f"{outfile}.h264", format="h264", resize=(640, 480)
        )
        camera.wait_recording(RECORD_TIME)
        camera.stop_recording()
        self.wrapH264(outfile)

    def wrapH264(self, videoName: str):
        mp4cmd: list = self.mp4ConvStr.split()
        mp4cmd.append(f"{videoName}.h264")
        mp4cmd.append(f"{videoName}.mp4")
        #    mp4cmd.append(devnull)
        # TODO run the conversion command in the background and delete following conversion
        subprocess.run(args=mp4cmd, stdout=subprocess.DEVNULL)
        os.remove(f"{videoName}.h264")


def readPirStatus():
    pir = False
    # First check that status file is recent - only read it if in past x mins
    statfile: os.stat_result = None
    try:
        statfile = os.stat(STATUS_FILE)
    except:
        statfile = None
    nowTime = datetime.now().timestamp()
    if statfile and (nowTime - statfile.st_mtime < MAX_LAST_UPDATE):
        with open(STATUS_FILE, "r", encoding="utf-8") as statusf:
            line: str = " "
            while line != "":
                line: str = statusf.readline()
                if line.startswith("PIR:"):
                    val = int(line[line.index(":") + 1])
                    if val:
                        pir = True
    return pir


def checkForPhotoCmd():
    # Check for photo command
    takePhoto: bool = False
    try:
        statfile = os.stat(TAKE_PHOTO_FILE)
        takePhoto = True
        os.remove(TAKE_PHOTO_FILE)
    except:
        takePhoto = False
    return takePhoto


# Check if any recognised mobile devices are on the network
def noOneHome():
    noOneHome = True
    try:
        statfile: os.stat_result = os.stat(DEVICES_FILE)
        # print(f"device file size: {statfile.st_size}")
        if statfile.st_size > 0:
            noOneHome = False
    except:
        noOneHome = True

    # Also register no one home if between 2300 and 0500
    # This ensures that capture is running overnight
    nowTime = datetime.now()
    if nowTime.hour > 23 or nowTime.hour < 5:
        noOneHome = True
    return noOneHome


# Start


def monitorAndRecord():
    camera: Camera = Camera(WEB_CAM)
    print("***Starting camera monitoring****")
    while True:
        # if no one home - start monitoring
        if noOneHome():
            # TODO - if no one home, take photo every hour?
            # Check pir is on
            while readPirStatus():
                # record in 15 second segments whilst pir status is on
                # and no one is home
                dateStr: str = datetime.now().strftime(DATE_FORMAT)
                camera.takePhoto(dateStr)
                camera.takeVideo(dateStr)
        if checkForPhotoCmd():
            dateStr: str = datetime.now().strftime(DATE_FORMAT)
            camera.takePhoto(dateStr)
        sleep(0.5)


if __name__ == "__main__":
    monitorAndRecord()
