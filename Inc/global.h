#define GPIOSEG GPIOA
#define A GPIO_PIN_0
#define B GPIO_PIN_1
#define C GPIO_PIN_2
#define D GPIO_PIN_3
#define E GPIO_PIN_4
#define F GPIO_PIN_5
#define G GPIO_PIN_6
#define DP GPIO_PIN_7

#define GPIODIG GPIOB
#define DIG1 GPIO_PIN_12
#define DIG2 GPIO_PIN_13
#define DIG3 GPIO_PIN_14
#define DIG4 GPIO_PIN_15

#define INPUT 2580
#define DELITEL 5
extern volatile int D1,D2,D3,D4,num,Kp,Uy,U0,Uoc,Xi,To,Tu,K1,K2,Xiold,DISP_MODE;
extern volatile short ADC_RESULT;


static void show(int digit)
{
  HAL_GPIO_WritePin(GPIOSEG, A|B|C|D|E|F|G|DP, GPIO_PIN_RESET);
  switch(digit)
  {
  case 0:
    HAL_GPIO_WritePin(GPIOSEG, A|B|C|D|E|F, GPIO_PIN_SET);//0
    break;
  case 1:
    HAL_GPIO_WritePin(GPIOSEG, B|C, GPIO_PIN_SET);//1
    break;
  case 2:
    HAL_GPIO_WritePin(GPIOSEG, A|B|G|E|D, GPIO_PIN_SET);//2
    break;
  case 3:
    HAL_GPIO_WritePin(GPIOSEG,A|B|C|D|G , GPIO_PIN_SET);//3
    break;
  case 4:
    HAL_GPIO_WritePin(GPIOSEG, F|G|B|C, GPIO_PIN_SET);//4
    break;
  case 5:
    HAL_GPIO_WritePin(GPIOSEG, A|F|G|C|D, GPIO_PIN_SET);//5
    break;
  case 6:
    HAL_GPIO_WritePin(GPIOSEG, A|F|G|C|D|E, GPIO_PIN_SET);//6
    break;
  case 7:
    HAL_GPIO_WritePin(GPIOSEG, A|B|C, GPIO_PIN_SET);//7
    break;
  case 8:
    HAL_GPIO_WritePin(GPIOSEG, A|B|C|D|E|F|G, GPIO_PIN_SET);//8
    break;
  case 9:
    HAL_GPIO_WritePin(GPIOSEG, A|B|C|D|F|G, GPIO_PIN_SET);//9
    break;
  case 10:
    HAL_GPIO_WritePin(GPIOSEG, A|B|C|E|F|G, GPIO_PIN_SET);//A
    break;
  case 11:
    HAL_GPIO_WritePin(GPIOSEG, C|D|E|F|G, GPIO_PIN_SET);//B
    break;
  case 12:
    HAL_GPIO_WritePin(GPIOSEG, A|D|E|F, GPIO_PIN_SET);//C
    break;
  case 13:
    HAL_GPIO_WritePin(GPIOSEG, B|C|D|E|G, GPIO_PIN_SET);//D
    break;
  case 14:
    HAL_GPIO_WritePin(GPIOSEG, A|D|E|F|G, GPIO_PIN_SET);//E
    break;
  case 15:
    HAL_GPIO_WritePin(GPIOSEG, A|E|F|G, GPIO_PIN_SET);//F
    break;
  case 16:
    HAL_GPIO_WritePin(GPIOSEG, A|B|C|D|E|F|G, GPIO_PIN_SET);
    break;
  }
}
static void select(int digit)
{
  HAL_GPIO_WritePin(GPIODIG, DIG1|DIG2|DIG3|DIG4, GPIO_PIN_SET);
  switch(digit)
	{
		case 0:
		{
                        HAL_GPIO_WritePin(GPIODIG, DIG1, GPIO_PIN_RESET);
                        break;
		}
		case 1:
		{
                        HAL_GPIO_WritePin(GPIODIG, DIG2, GPIO_PIN_RESET);
			break;
		}
		case 2:
		{
                        HAL_GPIO_WritePin(GPIODIG, DIG3, GPIO_PIN_RESET);
			break;
		}
		case 3:
		{
                        HAL_GPIO_WritePin(GPIODIG, DIG4, GPIO_PIN_RESET);
                        break;
		}
         }
}