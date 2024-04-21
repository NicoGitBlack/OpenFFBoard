<div align="center">
    <a href="https://github.com/Ultrawipf/OpenFFBoard">
        <img width="200" height="200" src="doc/img/ffboard_logo.svg">
    </a>
	<br>
	<br>
	<div style="display: flex;">
		<a href="https://discord.gg/gHtnEcP">
            <img src="https://img.shields.io/discord/704355326291607614">
		</a>
		<a href="https://github.com/Ultrawipf/OpenFFBoard/stargazers">
            <img src="https://img.shields.io/github/stars/Ultrawipf/OpenFFBoard">
		</a>
		<a href="https://github.com/Ultrawipf/OpenFFBoard/actions/workflows/build-firmware.yml">
            <img src="https://github.com/Ultrawipf/OpenFFBoard/actions/workflows/build-firmware.yml/badge.svg?branch=master">
		</a>
	</div>
</div>



# Open FFBoard
This fork is to support CyberGear Xiaomi motor (12N.m ~100€) 

More info about the CyberGear motor here:
	 - Github project https://github.com/search?q=repo%3Aproject-sternbergia%2Fcybergear_m5%20ADDR_IQ_REF&type=code
  	 	> Set a torque/current : 
     ```
     				#define ADDR_IQ_REF                0x7006
	 			#define CMD_RAM_WRITE                   18
     				void CybergearDriver::send_command(uint8_t can_id, uint8_t cmd_id, uint16_t option, uint8_t len, uint8_t * data)
				{
				  uint32_t id = cmd_id << 24 | option << 8 | can_id;
				  can_->send_message(id, data, len, true);
				  ++send_count_;
				}
     				void CybergearDriver::write_float_data(uint8_t can_id, uint16_t addr, float value, float min, float max)
				{
				  uint8_t data[8] = {0x00};
				  data[0] = addr & 0x00FF;
				  data[1] = addr >> 8;
				
				  float val = (max < value) ? max : value;
				  val = (min > value) ? min : value;
				  memcpy(&data[4], &value, 4);
				  send_command(can_id, CMD_RAM_WRITE, master_can_id_, 8, data);
				}
    ```
  	 - 

The Open FFBoard is an open source force feedback interface with the goal of creating a platform for highly compatible FFB simulation devices like steering wheels and joysticks.

This firmware is optimized for the Open FFBoard and mainly designed for use with DD steering wheels.
Remember this software is experimental and is intended for advanced users. Features may contain errors and can change at any time.

More documentation about this project is on the [hackaday.io page](https://hackaday.io/project/163904-open-ffboard).

The hardware designs are found under [OpenFFBoard-hardware](https://github.com/Ultrawipf/OpenFFBoard-hardware).

The GUI for configuration is found at [OpenFFBoard-configurator](https://github.com/Ultrawipf/OpenFFBoard-configurator).

These git submodules can be pulled with `git submodule init` and `git submodule update`

Updates often require matching firmware and GUI versions!

## Documentation
Documentation will be updated in the [GitHub Wiki](https://github.com/Ultrawipf/OpenFFBoard/wiki).

Available commands are listed on the [Commands wiki page](https://github.com/Ultrawipf/OpenFFBoard/wiki/Commands)

Code summary and documentation of the latest stable version is available as a [Doxygen site](https://ultrawipf.github.io/OpenFFBoard/doxygen/).

For discussion and progress updates we have a [Discord server](https://discord.com/invite/gHtnEcP).

### Extensions
The modular structure means you are free to implement your own main classes.
Take a look into the FFBoardMain and ExampleMain class files in the UserExtensions folder.
Helper functions for parsing CDC commands and accessing the flash are included.

The firmware is class based in a way that for example the whole main class can be changed at runtime and with it for example even the usb device and complete behavior of the firmware.

For FFB the motor drivers, button sources or encoders also have their own interfaces.

A unified command system supporting different interfaces is available and recommended for setting parameters at runtime. (see `CommandHandler.h` and the example mainclass)


### Copyright notice:
Some parts of this software may contain third party libraries and source code licenced under different terms.
The license applying to these files is found in the header of the file.
For all other parts in the `Firmware/FFBoard` folder the LICENSE file applies.
