# Setup
- `WiFi.ino` is to be ignored since a WiFi connecting function has already been declared in `tag-setup.ino`
- The system works on an anchor-and-tag system
  - For setup/usage, run `tag-setup.ino` on the device that is intended to be the tag, and `anchor-setup.ino` on the device that is intended to be the anchor
  - Calibration code: Run the calibration code to try to get an appropriate Adelay (antenna delay) value for your use case. Then, put this Adelay value into anchor-setup.ino. Make sure to run and compile calibration code for both the tag and the anchor together.
    - For the purposes of this project, the Adelay value has already been set.
    - Adjusting the Adelay value affects the accuracy of the reading.

**IMPORTANT NOTICE:** For DW1000 library, refer to the section *I am not getting the results I expected*
   
# Testing
- You should be able to achieve a minimum distance of 130m when anchor and tag are in line of sight
  - This is approximately the distance between Blocks 7/8 and the library.
- Obstacles like pillars and hiding behind a wall (from the anchor) should not be an issue
- Avoid testing with huge changes in elevation like a staircase. It will not work.
- Accuracy was tested to be in ±20cm margin

## I am not getting the results I expected
- https://github.com/Makerfabs/Makerfabs-ESP32-UWB
- From the above link, download the file `mf_DW1000.zip` and rename it to `DW1000.zip`. Once done, add it to your Arduino IDE library as a .zip file.
- Other libraries will not produce the same high distance results (40m max)
