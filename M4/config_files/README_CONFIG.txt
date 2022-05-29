The two variant.h and variant.cpp files located in this folder will enable you to use the M4_daq code and program the M4.

Use instructions:
1. Go to your file system (C:)
2. Go to /Users/[YourUser]
3. Go to /AppData/Local
4. Then you should see a folder titled Arduino15
5. Follow the file path Arduino15/packages/adafruit/hardware/samd/1.6.3/variants/grand_central_m4
6. Paste the two variant files from this github repo into the Arduino15/.../grand_central_m4 folder to replace the existing variant files.

This will add two new instances of serial to your M4's hardware abstraction layer (HAL), Serial2 and Serial3.

Serial2 is defined on pins 16 and 17. Serial 3 is defined in the variant.h folder.