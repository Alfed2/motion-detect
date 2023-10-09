# motion-detect
Raspberry Pi motion detector


<b>Device: Raspberry Pi 2 Model B Rev 1.1</b><br/>
&nbsp;  motion-h264-detect-opencv  ~ 4 FPS <br/>
&nbsp;  motion-h264-detect-vector  ~ 29 FPS<br/>

<b>Examples:<br/>
creates files in the directory(variable "DirSave"):</b> <br/>
&nbsp;  20231009_142258-f.jpg   - first frame motion detect<br/>
&nbsp;  20231009_142258-f-small.jpg<br/>
&nbsp;  20231009_142258-s.jpg   - frame after ~ 1.5sec from the motion detect<br/>
&nbsp;  20231009_142258-s-small.jpg<br/>
&nbsp;  20231009_142258.mp4     - Video motion detect<br/>
<br/>
<b>Start the alert in ~1.5 sec:</b><br/>
&nbsp; running script send.sh, parameter($1): "varable Dirsave"+20231009_142258-s.jpg<br/>
&nbsp; running script after video file recording is complete, parameter($1): "varable Dirsave"+20231009_142258.mp4<br/>


Build
-----
0. Install pre-required packages
   
    libopencv-dev ffmpeg ffmpeg-devel

1. Place  Raspberry Pi userland project in /home/pi/src/raspberrypi/userland
    
    $ mkdir -p /home/pi/src/raspberrypi
    
    $ cd /home/pi/src/raspberrypi
        
    $ git clone --depth 1 https://github.com/raspberrypi/userland.git


2. Build pre-required libraries
    
    $ make -C /opt/vc/src/hello_pi/libs/vgfont
    

3. Build project 

    $ mkdir build
    
    $ cd build
    
    $ cmake ../
    
    $ make 
    
  
