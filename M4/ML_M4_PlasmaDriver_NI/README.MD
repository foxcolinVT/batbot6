This document serves as a jumping off point for the embedded data acquisition systems within the bat bot. The Adafruit GrandCentral M4, and M4 express contain the SAMD51 microcontroller. 

It is recommended that you use Visual Studio Code with the PlatformIO extension to interface with the board. I do not recommend using the Arduino IDE for working with the main code base because Arduino requires the .ino file extension and typically forces you into writing all code into one file. That being said, I would use the Arduino IDE as a standalone test environment. When contributing new code, I would HIGHLY recommend building your system as a standalone unit before integrating into the rest of the code. As of now we do not have access to a debugger, so if something is causing the MCU to throw an un handleable exception, we cant really gain much insight as to why it happened.

As I will get into later, the SAMD51 gives the capability of multiple clock domains and dedicated peripheral busses,  which are useful because we can run our peripherals like ADCs, DACs, PWM controllers, etc., asynchronously from the CPU, alleviating it from the clock cycles it would take to move data from one peripheral to another or to and from memory. Because of this, we have to synchronize clock domains in order to modify some peripheral registers in our code. This is fairly analogous to how multi threading is handled with mutex locks. Basically, if we try to access data that is being written by another process in a different clock domain, how can we determine whether the data we are reading is the right data? In some cases, not synchronizing clocks before or after a R/W operation, will cause the CPU will throw an unrecoverable exception and halt. 

=======================================================================================================

Other things which may cause unrecoverable exceptions:

- Libraries: 
I am not opposed to using libraries where they are applicable, however, this code configures many parts of the MCU manually, which a library may overwrite causing a many number of problems. If you do use a library, try to find the most lightweight implementation and use that (we want as little dependencies as possible to keep the code 
concise).

- Handling interrupts:
Due to the inherent nature of interrupts, if we modify some data within an interrupt handler, we have to be careful about accessing that data in our application loop. Attempting to access data data of this nature can and will lead to unrecoverable exceptions. To avoid this:

```
data_ready = false

some_handler:
	blah, blah blah
	...
	data_ready = true

loop:
	if data_ready
	then blah
	     .... 
	     data_ready = false
```
It should be clear that this code will only R/W data also used by a handler when the associated interrupt is not being handled. 

- Serial:
This is a simple one to avoid. Arduino's serial object wants to set some of its own clock settings during setup, so when using serial for debugging, call Serial.begin as the last call in the setup code. 

- Reading and writing registers
When working with registers (even when using libraries it is very common that you will have to directly read and write to registers), make sure to use the predefined bit masks i.e. please refrain from setting registers like this:
```
some_reg = (1 << 10) | (1 << 5) | (1 << 8) | ...

some_reg = 0x45
```
Although the top one is slightly better than doing that math by hand and just setting the register equal to the resulting value, both are extremely ambiguous and will require you and others to spend time determining what these bits are actually setting. Instead, the MCU manufacturer provides hardware definitions already. Although these will already exist on your computer, download a copy **HERE** and keep them handy. 

=======================================================================================================

You will also need the datasheet for the SAMD51 board. Fair warning, this datasheet is roughly 1000 pages long and can be very daunting at first. It explains every single nook and cranny of this MCU in immense detail and should hopefully serve as the number one resource when developing code. That being said, the datasheet explains a few things very poorly (rather the implementation of some things is not very intuitive). However, this is for the most part not true and the organization of the MCU may just take some getting used to. 

Each section of the datasheet will start with the general information about the system. This will include, for example, which registers need to be synced when being accessed or written, which other systems have direct interfacing, registers which have special access permissions (like readonly, writeonly or none), among other things. It will then explain the functional description of the system (truly a good review of digital systems) and will end with a table of all contained registers and a summary for each register. 

Names for control lines, multiplexers, constants and registers will follow a similar format to what is seen in the hardware definitions discussed earlier, however the hardware definitions are prefixed with the peripheral name and associated register to avoid naming conflicts. 

Do not use hardware definitions ending in "_val". This is the raw value to be set and hasn't been shifted to set the right bits. There should be a macro that has already shifted the value for you. 

I initially had a hard time translating what the datasheet says to actual code, but after working with it more, patterns emerged and the datasheet became a much more powerful tool. 

=======================================================================================================

The main driver code exists in ML_M4_PlasmaDriver_NI.cpp and hpp. There is also a utils.cpp, hpp. The utils serves as a place to put hardware configuration code that is typically redundant across all peripherals. For example, almost all peripherals can be enabled, disabled and software reset. In other words, if it is code that is not important in the explanation of the system, it should go into the utils. 

Although this is said in about every coding class ever, please do not over or under compartmentalize code, i.e. no ridiculously long ambiguous functions, but also don't make everything a function call because not only is that harder to understand, but it wastes CPU cycles and is thus inefficient. 

Please also read into C macros. Not everything has to be declared as a variable. This is also inefficient. Instead, macro definitions are resolved by the precompiler and replaces all macro definitions in the code with their raw defined value before the code is sent to the compiler. 

=======================================================================================================

Functional Description

<img width="466" alt="Screen Shot 2022-10-13 at 10 46 31 PM" src="https://user-images.githubusercontent.com/29962669/195752257-9de46abd-a966-4504-82bf-fcdf6d332e68.png">




