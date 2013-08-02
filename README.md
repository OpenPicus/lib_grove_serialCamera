lib_grove_serialCamera
======================

Flyport library for Grove Serial Camera sensor, released under GPL v.3.

Serial Camera v0.9b is a JPEG color camera module easy for MCU use.It has integrated image processing DSP to generate 320*240 or 640*480 JPEG image without thumbnail information, captured picture will be stored in internal buffer and transferred via UART port. <br>
Features<br>
 - 640x480/320x240(default) resolution<br>
 - JPEG compressed image without Thumbnail Information<br>
<br>
The communication with Flyport is realized through a specific library only needing few commands to make it works.<br>
An example of usage follows. More info on wiki.openpicus.com. 
1) import files inside Flyport IDE using the external libs button.
2) if you want to use also uSD card for image storage import FAT file system libraries (wiki.openpicus.com)
