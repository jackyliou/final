#include "mbed.h"
#include "mbed_rpc.h"
#include "fsl_port.h"
#include "fsl_gpio.h"
#include "bbcar.h"
#include "bbcar_rpc.h"

// For OpenMV
Serial uart(D1, D0); //tx,rx
// For serv control
Ticker servo_ticker;
PwmOut pin9(D13), pin8(D10);
BBCar car(pin8, pin9, servo_ticker);
// For getting ping data
DigitalOut redLED(LED1);
DigitalInOut pin10(D8);
parallax_ping ping(pin10);
// Xbee transfer
RawSerial xbee(D12, D11);
Thread t;
EventQueue queue(16 * EVENTS_EVENT_SIZE);
int mission_gap = 0;
int co = 0;
char behavior = 'S';

// sub-functions
void Comunication(void);
void Ping_data_check(void);
void matrix_get(void);
void identify_get(void);
void turnaway(int turnway_lr);
void ping_guilde(int close, int speed);

int main()
{
  // initialization setting
  xbee.baud(9600);
  uart.baud(9600);
  redLED = 1;
  // start xbee sending thread
  t.start(callback(&queue, &EventQueue::dispatch_forever));
  queue.call(&Comunication);
  // -----maze route----- //
  // straight until wall
  wait(2);
  behavior = 'S';
  ping_guilde(30, 100);
  // matrix getting
  matrix_get();
  wait(2);
  behavior = 'S';
  ping_guilde(20, 100);
  // turn left
  wait(2);
  behavior = 'L';
  turnaway(1);
  // straight until wall
  wait(1);
  behavior = 'S';
  car.goStraight(100);
  wait(4);
  car.stop();
  // -----  misson1 ----- //
  // turn back left
  behavior = 'L';
  turnaway(0);
  // backforward
  wait(1);
  behavior = 'B';
  car.goStraight(-100);
  while (1)
  {
    if ((float)ping > 40)
    {
      car.stop();
      break;
    }
    wait_ms(10);
  }
  // frontforward *************************
  wait(1);
  behavior = 'S';
  ping_guilde(30, 100);
  // identify getting
  identify_get();
  wait(1);
  behavior = 'S';
  ping_guilde(15, 100);
  // turn right
  wait(1.5);
  behavior = 'R';
  turnaway(0);
  // straight until wall
  wait(1);
  behavior = 'S';
  car.goStraight(100);
  wait(2.5);
  car.stop();
  // -----maze route----- //
  // turn right
  wait(1);
  behavior = 'R';
  turnaway(0);
  // straight until wall
  wait(2);
  behavior = 'S';
  ping_guilde(15, 100);
  // turn right
  behavior = 'R';
  wait(1);
  turnaway(0);
  // -----  misson2 ----- //
  // straight until wall
  wait(2);
  behavior = 'S';
  ping_guilde(60, 100);
  // turn right
  wait(1);
  behavior = 'R';
  turnaway(0);
  // ping_ceheck task
  wait(3);
  Ping_data_check();
  // turn back right
  behavior = 'L';
  wait(1);
  turnaway(1);
  // straight until wall
  wait(1);
  behavior = 'S';
  ping_guilde(15, 100);
  // -----maze route----- //
  // turn right
  wait(1);
  behavior = 'R';
  turnaway(0);
  // go straight to finish
  wait(1);
  behavior = 'S';
  car.goStraight(100);
}
// Xbee keep-comunication
void Comunication(void)
{
  while (1)
  {
    if (mission_gap == 1)
    {
      wait(1);
    }
    if (mission_gap == 0)
    {
      // behavior log: S for straight, L for left turn, R for left turn, B for back
      xbee.printf("%d %c\r\n", co, behavior);
      co++;
      wait(1);
    }
  }
}
// getting ping data
void Ping_data_check(void)
{
  float obj_data[3];
  int result = 1;
  float dif1, dif2;
  redLED = 0;
  mission_gap = 1;
  xbee.printf("80663\r\n");
  // data grabbing
  obj_data[0] = (float)ping;
  car.turn(30, 0.01);
  wait(0.5);
  car.stop();
  wait(2);
  obj_data[1] = (float)ping;
  car.turn(-30, 0.01);
  wait(1);
  car.stop();
  wait(2);
  obj_data[2] = (float)ping;
  car.turn(30, 0.01);
  wait(0.5);
  car.stop();
  // calculate
  dif1 = obj_data[1] - obj_data[0];
  dif2 = obj_data[2] - obj_data[0];
  if (dif1 < -0.8)
  {
    if (dif2 > 0.7)
      if (dif1 > -5)
        if (dif1 < -1.9)
          if (dif2 > 2)
            result = 2;
        else
          result = 1;
  }
  if (dif1 < -4)
    if(dif2 > 0.8)
      if(dif2 < 4)
        result = 4;
  else
    result = 3;
  // output result to xbee
  if (result == 1)
    xbee.printf("square\r\n");
  if (result == 2)
    xbee.printf("slope\r\n");
  if (result == 3)
    xbee.printf("tip\r\n");
  if (result == 4)
    xbee.printf("indent\r\n");
  mission_gap = 0;
  redLED = 1;
}
// UART command-Matrix
void matrix_get(void)
{
  mission_gap = 1;
  redLED = 0;
  // send quest
  char s[10];
  sprintf(s, "matrix");
  uart.puts(s);
  wait(5);
  // fetch information
  if (uart.readable())
  {
    char temp;
    char rev[20];
    int counter = 1;
    xbee.printf("80662\r\n");
    while (1)
    {
      temp = uart.getc();

      if (temp == '\0')
      {
        temp = uart.getc();
      }
      else if (temp != '\r')
      {
        rev[counter] = temp;
        counter++;
      }
      else
      {
        break;
      }
    }
    for (int i = 1; i < counter; i++)
    {
      xbee.putc(rev[i]);
    }
    counter = 1;
    xbee.printf("\r\n");
  }
  mission_gap = 0;
  redLED = 1;
}
// UART command-identify
void identify_get(void)
{
  mission_gap = 1;
  redLED = 0;
  // send quest
  char s[10];
  sprintf(s, "identify");
  uart.puts(s);
  wait(5);
  // fetch information
  if (uart.readable())
  {
    char recv = uart.getc();
    xbee.printf("80661\r\n");
    xbee.putc(recv);
    xbee.printf("\r\n");
  }
  mission_gap = 0;
  redLED = 1;
}
// going turn: 1 for left ; 0 for right
void turnaway(int turnway_lr)
{
  // left
  if (turnway_lr == 1)
  {
    car.turn(100, -0.45);
    wait(1);
    car.stop();
  }
  // right
  if (turnway_lr == 0)
  {
    car.turn(100, 0.45);
    wait(1);
    car.stop();
  }
}
// ping guilde
void ping_guilde(int close, int speed)
{
  car.goStraight(speed);
  while (1)
  {
    if ((float)ping < close)
    {
      car.stop();
      break;
    }
    wait_ms(10);
  }
}