#ifndef __DISPLAY_H__
#define __DISPLAY_H__

#define DISPLAY_TIME_X     0
#define DISPLAY_TIME_Y     0

#define DISPLAY_WORK_STATUS_X 12
#define DISPLAY_WORK_STATUS_Y 5

#define BATTERY_VALUE_FULL    1276
#define BATTERY_VALUE_TWO     1170
#define BATTERY_VALUE_ONE     1099
#define BATTERY_VALUE_ZERO    1028

void Display_Process_Task(void const *argument);

#endif

