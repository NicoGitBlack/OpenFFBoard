/*
 * LocalButtons.h
 *
 *  Created on: 09.02.2020
 *      Author: Yannick
 */

#ifndef LOCALBUTTONS_H_
#define LOCALBUTTONS_H_

#include <ButtonSource.h>
#include "ChoosableClass.h"
#include "CommandHandler.h"
#include "constants.h"

class LocalButtons: public ButtonSource,CommandHandler{
public:
	LocalButtons();
	virtual ~LocalButtons();
	uint8_t readButtons(uint64_t* buf);

	const ClassIdentifier getInfo();
	static ClassIdentifier info;
	static bool isCreatable() {return true;};

	static constexpr uint16_t maxButtons{BUTTON_PINS};

	ParseStatus command(ParsedCommand* cmd,std::string* reply);

	void saveFlash(); 		// Write to flash here
	void restoreFlash();	// Load from flash

	static inline bool readButton(int button_num) {
		if (button_num >= maxButtons || button_num < 0) {
			return false;
		}
		return HAL_GPIO_ReadPin(button_ports[button_num], button_pins[button_num]);
	}

private:
	uint32_t mask = 0xff;
	void setMask(uint32_t mask);
	bool polarity = false;
	static constexpr uint16_t button_pins[8] {DIN0_Pin,DIN1_Pin,DIN2_Pin,DIN3_Pin,DIN4_Pin,DIN5_Pin,DIN6_Pin,DIN7_Pin};
	static constexpr GPIO_TypeDef* button_ports[8] {DIN0_GPIO_Port,DIN1_GPIO_Port,DIN2_GPIO_Port,DIN3_GPIO_Port,DIN4_GPIO_Port,DIN5_GPIO_Port,DIN6_GPIO_Port,DIN7_GPIO_Port};
};

#endif /* LOCALBUTTONS_H_ */
