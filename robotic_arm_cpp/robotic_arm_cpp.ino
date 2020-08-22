/*
 * Intended to use with 74hc595 shift registers
 * Each shift register can control up to two 28byj stepper motors with ULN2003 driver.
 */
#include <Wire.h>
#include <EEPROM.h>
#define _DOF_NUMBER             3
#define _EEPROM_ADDR_TASK_COUNT 0x00
#define _EEPROM_ADDR_BEGIN_DATA 0x01
#define _EEPROM_DEVICE_ADDRESS  0x50
#define _STEP_DELAY             6
#define _TASKQUEUE_MODE_CONFIG  0
#define _TASKQUEUE_MODE_RUN     1

/*
 * Class responsible to save and restore data about tasks.
 * Original name was EEPROMClass, but this name is already in use on EEPROM.h.
 */
class RobotEEPROMClass
{
private:
    byte eeprom_location=0;
    void writeByteInternalEEPROM(unsigned int _addr, byte _data) {
        EEPROM.write(_addr,_data);
        delay(10);
    }
    
    byte readByteInternalEEPROM(unsigned int _addr) {
        byte ret;
        ret=EEPROM.read(_addr);
        delay(10);
        return ret;
    }

    void writeByteI2C(unsigned int _addr, byte _data) {
        // I2C is intended to use with AT24C256
        Wire.beginTransmission(_EEPROM_DEVICE_ADDRESS);
        Wire.write(_addr >> 8);
        Wire.write(_addr & 0xFF);
        Wire.write(_data);
        Wire.endTransmission();
        delay(10);
    }

    byte readByteI2C(unsigned int _addr) {
        // I2C is intended to use with AT24C256
        Wire.beginTransmission(_EEPROM_DEVICE_ADDRESS);
        Wire.write(_addr >> 8);
        Wire.write(_addr & 0xFF);
        Wire.endTransmission();
        delay(10);
        Wire.requestFrom(_EEPROM_DEVICE_ADDRESS, 1);
        return Wire.read();
    }
public:
    RobotEEPROMClass() {}

    void writeByte(unsigned int _addr, byte _data) {
        if(this->eeprom_location==0) writeByteInternalEEPROM(_addr,_data);
        if(this->eeprom_location==1) writeByteI2C(_addr,_data);
    }

    byte readByte(unsigned int _addr) {
        byte ret;
        if(this->eeprom_location==0) ret=readByteInternalEEPROM(_addr);
        if(this->eeprom_location==1) ret=readByteI2C(_addr);
        return ret;
    }

    void useInternal() {
        this->eeprom_location=0;
    }

    void useI2C() {
        this->eeprom_location=1;
    }
};

/*
 * Send data to steppers and make the movement happen :)
 */
class StepperControlClass
{
private:
    unsigned char current_coil[_DOF_NUMBER] = { 0, 0, 0 };
    int steps_from_start_position[_DOF_NUMBER] = { 0, 0, 0 };
    unsigned int current_count[_DOF_NUMBER] = { 0, 0, 0 };
    unsigned char direction[_DOF_NUMBER] = { 0, 0, 0 };
    int step_delay[_DOF_NUMBER] = { 0, 0, 0 };
    unsigned long next_step[_DOF_NUMBER] = { 0, 0, 0 };
    unsigned char full_step[4] = { 0x3, 0x6, 0xC, 0x9 };
    unsigned int steps_to_finish = 0;
    int reset_pin;
    int latch_pin;
    int clock_pin;
    int data_pin;
    void sendDataToShiftRegister()
    {
        if (this->reset_pin) {
            digitalWrite(this->reset_pin, LOW);
            digitalWrite(this->reset_pin, HIGH);
        }

        for (int i = _DOF_NUMBER - 1; i >= 0; i--) {
            for (int b = 0; b < 4; b++) {
                digitalWrite(this->data_pin, ((0x08 >> b) &(this->full_step[this->current_coil[i]])) ? HIGH : LOW);
                digitalWrite(this->clock_pin, HIGH);
                digitalWrite(this->clock_pin, LOW);
            }
        }

        digitalWrite(this->latch_pin, HIGH);
        digitalWrite(this->latch_pin, LOW);
    }
public:
    StepperControlClass() {}

    void setShiftRegisterPinout(int _data_pin, int _clock_pin, int _latch_pin, int _reset_pin) {
        this->reset_pin = _reset_pin;
        this->latch_pin = _latch_pin;
        this->clock_pin = _clock_pin;
        this->data_pin = _data_pin;
    }

    void setStepperValues(int _stepper_id, unsigned int _step_count, int _step_delay, unsigned char _direction) {
        this->current_count[_stepper_id] = _step_count;
        this->direction[_stepper_id] = _direction;
        this->step_delay[_stepper_id] = _step_delay;
        this->steps_to_finish += _step_count;
        if (_direction == 0) {
            this->steps_from_start_position[_stepper_id] += _step_count;
        } else {
            this->steps_from_start_position[_stepper_id] -= _step_count;
        }
    }

    void backToInitialPosition() {
        for (int i = 0; i < _DOF_NUMBER; i++) {
            this->setStepperValues(i, abs(this->steps_from_start_position[i]), _STEP_DELAY, (this->steps_from_start_position[i] <= 0 ? 0 : 1));
        }

        this->runSteppers();
    }

    void runSteppers() {
        byte data_changed = 0;
        unsigned long mi = 0;

        while (this->steps_to_finish) {
            for (int i = 0; i < _DOF_NUMBER; i++) {
                if (this->current_count[i] > 0) {
                    mi = millis();
                    if (this->next_step[i] <= mi) {
                        this->next_step[i] = mi + this->step_delay[i];
                        if (direction[i] == 0) {
                            this->current_coil[i]++;
                            if (this->current_coil[i] == 4) this->current_coil[i] = 0;
                        } else {
                            if (this->current_coil[i] == 0) this->current_coil[i] = 4;
                            this->current_coil[i]--;
                        }
                        this->steps_to_finish--;
                        this->current_count[i]--;
                        data_changed = 1;
                    }
                }
            }

            if (data_changed) {
                this->sendDataToShiftRegister();
                data_changed = 0;
            }
        }
    }
};

/*
 * Read encoders and implements some methods to control pulse count and direction.
 */
class EncoderKY040 {
private:
    int sensor_id;
    int clk;
    int dt;
    int value = 0; //current_value
    int last_value = 0;//last_value
    int counter = 0; //pulse_counter
    byte inc=1; //increment_value
    byte movement_direction=0;
public:
    EncoderKY040() {}

    EncoderKY040(int _clk, int _dt) {
        this->clk = _clk;
        this->dt = _dt;
    }

    void setCLKPin(int _clk) {
        this->clk=_clk;
    }

    void setDTPin(int _dt) {
        this->dt=_dt;
    }

    void readSensor() {
        this->value = (digitalRead(this->clk) << 1) | digitalRead(this->dt);
        if(this->value!=this->last_value) {
            if ((this->value == 3 && this->last_value == 2) || (this->value == 0 && this->last_value == 1)) {
                movement_direction=1; // clockWise
                this->increaseCounter();
            }
            if ((this->value == 3 && this->last_value == 1) || (this->value == 0 && this->last_value == 2)) {
                movement_direction=2; // antiClockWise
                this->decreaseCounter();
            }
            this->last_value=this->value;
            delay(5);
        }
    }

    void setIncrement(int _inc) {
        this->inc=_inc;
    }

    void increaseCounter(int _n) {
        this->counter+=_n;
    }

    void increaseCounter() {
        this->counter+=this->inc;
    }

    void decreaseCounter(int _n) {
        this->counter-=_n;
    }

    void decreaseCounter() {
        this->counter-=this->inc;
    }

    void setCounter(int _n) {
        this->counter=_n;
    }

    void resetCounter() {
        this->counter=0;
    }

    void resetSensorDirection() {
        this->movement_direction=0;
    }

    int getCounter() {
        return this->counter;
    }

    byte clockwiseChangeDetected() {
        byte r;
        r=this->movement_direction;
        this->movement_direction=0;
        return r==1;
    }

    byte anticlockwiseChangeDetected() {
        byte r;
        r=this->movement_direction;
        this->movement_direction=0;
        return r==2;
    }

    byte changeDetected() {
        return this->movement_direction;
    }

    byte getLastDirection() {
        return this->movement_direction;
    }
};

/*
 * Store data about task that would be executed by a single stepper motor
 */
class StepperTasks {
private:
    unsigned char direction = 0;
    unsigned int step_count = 0;
    unsigned int step_delay = 0;
public:
    StepperTasks() {}

    setStepperTaskValues(unsigned int _direction, unsigned int _step_count, unsigned int _step_delay) {
        this->direction = _direction;
        this->step_count = _step_count;
        this->step_delay = _step_delay;
    }

    unsigned int getStepCount() {
        return this->step_count;
    }

    unsigned int getStepDelay() {
        return this->step_delay;
    }

    unsigned char getDirection() {
        return this->direction;
    }
};

/*
 * Group of stepper tasks. A stepper task group contains information about all steppers on robot arm.
 */
class TaskGroup {
private:
    byte task_id;
    unsigned char one_time_execution = 0;
    unsigned char task_executed = 0;
    StepperTasks st[_DOF_NUMBER];

public:
    TaskGroup() {
        for (unsigned char i = 0; i < _DOF_NUMBER; i++) st[i] = StepperTasks();
    }

    TaskGroup(byte _task_id) {
        for (unsigned char i = 0; i < _DOF_NUMBER; i++) st[i] = StepperTasks();
        this->task_id=_task_id;
    }

    void setExecuted() {
        this->task_executed = 1;
    }

    void setOneTimeExecution() {
        this->one_time_execution = 1;
    }

    int isRunnable() {
        return this->one_time_execution == 0 || (this->one_time_execution == 1 && this->task_executed == 0);
    }

    void addStepperTask(unsigned char id_stepper, unsigned char direction, unsigned int step_count, unsigned int step_delay) {
        this->st[id_stepper].setStepperTaskValues(direction, step_count, step_delay);
    }

    int getStepperTaskCount(int _id) {
        return st[_id].getStepCount();
    }

    int getStepperTaskDirection(int _id) {
        return st[_id].getDirection();
    }

    int getStepperTaskDelay(int _id) {
        return st[_id].getStepDelay();
    }

    byte getTaskGroupId() {
        return this->task_id;
    }
};

/*
 * Queue of taskgroups that will be executed in a circular way.
 */
class TaskQueue
{
private:
    char encoder_count = 0;
    int create_task_button = 13;
    int run_taskqueue_button = 12;
    unsigned char mode = _TASKQUEUE_MODE_CONFIG;
    unsigned char task_group_count = 0;
    int steps_to_first_task[3] = { 0, 0, 0 };
    StepperControlClass stepperControl;
    EncoderKY040 EncoderList[_DOF_NUMBER];
    TaskGroup tg[20];
    RobotEEPROMClass eeprom;

    void createTaskGroup(int _task_id) {
        tg[_task_id] = TaskGroup(_task_id);
    }

    void addStepperTask(unsigned char id_grupo, unsigned char id_stepper, unsigned char direction, unsigned int step_count, unsigned int step_delay) {
        tg[id_grupo].addStepperTask(id_stepper, direction, step_count, step_delay);
    }

    void saveTaskQueueToEEPROM() {
        int byte_count=0;
        int stepper_task_count=0;
        union {
          byte data[6];
          struct {
            byte task_group;
            byte stepper_id;
            byte step_delay;
            unsigned int step_count;
            byte direction;
          };
        } eeprom_data;
        for(int t=0;t<task_group_count;t++) {
            for(int s=0;s<_DOF_NUMBER;s++) {
                eeprom_data.task_group=t;
                eeprom_data.stepper_id=s;
                eeprom_data.step_delay=tg[t].getStepperTaskDelay(s);
                eeprom_data.step_count=tg[t].getStepperTaskCount(s);
                eeprom_data.direction=tg[t].getStepperTaskDirection(s);
                for(int b=0;b<6;b++) {
                    eeprom.writeByte(_EEPROM_ADDR_BEGIN_DATA+byte_count,eeprom_data.data[b]);
                    byte_count++;
                }
                stepper_task_count++;
                eeprom.writeByte(_EEPROM_ADDR_TASK_COUNT,stepper_task_count);
            }
        }
    }

    void createTaskQueueFromEEPROM() {
        union {
          byte data[6];
          struct {
            byte task_group;
            byte stepper_id;
            byte step_delay;
            unsigned int step_count;
            byte direction;
          };
        } eeprom_data;
        byte task_count=0;
        byte last_task=0xFF;
        int byte_count=0;
        task_count=eeprom.readByte(_EEPROM_ADDR_TASK_COUNT);
        for(int t=0;t<task_count;t++) {
            for(int b=0;b<6;b++) {
                eeprom_data.data[b]=eeprom.readByte(_EEPROM_ADDR_BEGIN_DATA + byte_count);
                byte_count++;
            }
            if(last_task!=eeprom_data.task_group) {
                this->createTaskGroup(eeprom_data.task_group);
                this->task_group_count++;
                if (eeprom_data.task_group == 0) tg[0].setOneTimeExecution();
                last_task=eeprom_data.task_group;
            }
            this->addStepperTask(eeprom_data.task_group, eeprom_data.stepper_id, eeprom_data.direction, eeprom_data.step_count, eeprom_data.step_delay);
        }
        this->mode=_TASKQUEUE_MODE_RUN;
    }

    void createStepperTask() { // createStepperTask
        unsigned char task_group;
        this->createTaskGroup(task_group_count);
        task_group=tg[task_group_count].getTaskGroupId();
        task_group_count++;
        for (int i = 0; i < _DOF_NUMBER; i++) {
            this->addStepperTask(task_group, i, (EncoderList[i].getCounter() <= 0 ? 1 : 0), abs(EncoderList[i].getCounter()), _STEP_DELAY);
            if (task_group != 0) steps_to_first_task[i] += (EncoderList[i].getCounter() *-1);
            EncoderList[i].resetCounter();
        }

        if (task_group == 0) tg[0].setOneTimeExecution();
    }

    void checkButtonPressedCreateTask() {
        if (digitalRead(this->create_task_button) == 1) {
            createStepperTask();
            delay(2000);  //Avoid push-button bouncing
        }
    }

    void checkButtonPressedRunTaskQueue() {
          if (digitalRead(this->run_taskqueue_button) == 1) {
              mode = _TASKQUEUE_MODE_RUN;
              for (int i = 0; i < _DOF_NUMBER; i++) {
                  EncoderList[i].setCounter(steps_to_first_task[i]);
              }
  
              createStepperTask();
              delay(2000);  //Avoid push-button bouncing
              saveTaskQueueToEEPROM();
              stepperControl.backToInitialPosition();
          }
    }

    void configureStepperTask() {
        int inc_step = 25;
        while(mode==_TASKQUEUE_MODE_CONFIG) {
            for (int i = 0; i < _DOF_NUMBER; i++) {
                EncoderList[i].readSensor();
                EncoderList[i].setIncrement(inc_step);
                if(EncoderList[i].changeDetected()) {
                    stepperControl.setStepperValues(i, inc_step, _STEP_DELAY, EncoderList[i].getLastDirection()==1?0:1);
                    stepperControl.runSteppers();
                    EncoderList[i].resetSensorDirection();
                }
            }

            checkButtonPressedCreateTask();
            checkButtonPressedRunTaskQueue();
    
        }
    }

    void runTasks() {
        while(mode==_TASKQUEUE_MODE_RUN) {
            for (int i = 0; i < task_group_count; i++) {
                if (tg[i].isRunnable()) {
                    tg[i].setExecuted();
                    for (int s = 0; s < _DOF_NUMBER; s++) {
                        stepperControl.setStepperValues(s, tg[i].getStepperTaskCount(s), tg[i].getStepperTaskDelay(s), tg[i].getStepperTaskDirection(s));
                    }

                    stepperControl.runSteppers();
                }
            }
        }
    }

    byte loadFromEEPROMOnBoot() {
      byte button_down=0;
      delay(1000);
      button_down=(digitalRead(this->create_task_button)<<1)|digitalRead(this->run_taskqueue_button);
      if(button_down==3) {
        delay(1000);
        button_down=(digitalRead(this->create_task_button)<<1)|digitalRead(this->run_taskqueue_button);
        delay(2000);
      }
      if(button_down!=3) button_down=0;
      return button_down;
    }

public:
    TaskQueue() {
        eeprom=RobotEEPROMClass();
        eeprom.useInternal();
    }

    void createEncoder(int _clk, int _dt) { //addEncoder
        EncoderList[encoder_count] = EncoderKY040(_clk, _dt);
        encoder_count++;
    }

    void setShiftRegisterPinout(int _data_pin, int _clock_pin, int _latch_pin, int _reset_pin) {
        stepperControl.setShiftRegisterPinout(_data_pin, _clock_pin, _latch_pin, _reset_pin);
    }

    void setPinCreateTaskButton(int _pin) {
        this->create_task_button = _pin;
    }

    void setPinRunTaskQueueButton(int _pin) {
        this->run_taskqueue_button = _pin;
    }

    void init() {
        if (loadFromEEPROMOnBoot()==3) this->createTaskQueueFromEEPROM();
        if (mode == _TASKQUEUE_MODE_CONFIG) configureStepperTask();
        if (mode == _TASKQUEUE_MODE_RUN) runTasks();
    }
};

TaskQueue tq;

void setup() {
    Wire.begin();
    pinMode(2, INPUT);
    pinMode(3, INPUT);
    pinMode(4, INPUT);
    pinMode(5, INPUT);
    pinMode(6, INPUT);
    pinMode(7, INPUT);
    pinMode(8, OUTPUT);
    pinMode(9, OUTPUT);
    pinMode(10, OUTPUT);
    pinMode(11, OUTPUT);
    pinMode(12, INPUT);
    pinMode(13, INPUT);
    tq = TaskQueue();
    tq.createEncoder(6, 7);
    tq.createEncoder(4, 5);
    tq.createEncoder(2, 3);
    tq.setShiftRegisterPinout(8, 10, 9, 11);
    tq.setPinCreateTaskButton(13);
    tq.setPinRunTaskQueueButton(12);
}

void loop() {
    tq.init();
}
