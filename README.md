# motion-detect
Raspberry Pi motion detector

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
    
  
