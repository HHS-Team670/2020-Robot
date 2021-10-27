import os
import subprocess
# os.system('START "" cd C:\\Users\\Driver\\Documents\\GitHub\\FRCDashboard & START "" npm start')

# subprocess.Popen(["cd", "C:\\Users\\Driver\\Documents\\GitHub\\FRCDashboard"])
# subprocess.Popen(["npm start"], stdout=subprocess.PIPE, stderr=subprocess.PIPE)

# subprocess.Popen("npm start", cwd="C:\\Users\\Driver\\Documents\\GitHub\\FRCDashboard")
# os.chdir("C:\\Users\\Driver\\Documents\\GitHub\\2020-Robot\\Dashboard2020")
os.chdir("C:\\Users\\Driver\\Downloads\\2020-Robot-dev\\2020-Robot-dev\\Dashboard2020")
os.system('START "" npm start')
os.system('START "" C:\\gstreamer\\1.0\\x86_64\\bin\\gst-launch-1.0.exe udpsrc port=5800 caps="application/x-rtp" ! rtpmp4vdepay ! avdec_mpeg4 ! fpsdisplaysink &')
os.system('START "" C:\\gstreamer\\1.0\\x86_64\\bin\\gst-launch-1.0.exe udpsrc port=5801 caps="application/x-rtp" ! rtpmp4vdepay ! avdec_mpeg4 ! fpsdisplaysink &')

