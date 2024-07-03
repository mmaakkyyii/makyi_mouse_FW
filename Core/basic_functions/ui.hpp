#ifndef _UI_H_
#define _UI_H_

class UI{
public:
	UI():RBLED_value(0){};
	void SetLED(int led_data);
	void SetRBLED(int value);
	void Init();
	void Update();
	int GetSW1();
	int GetSW2();
	int GetSW3();
private:
	int RBLED_value;
};

#endif //_UI_H_
