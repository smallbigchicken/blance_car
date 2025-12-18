#include "LCD_gui.h"

void Lcd_test(void)
{
    LCD_Init(L2R_U2D,1000);
    LCD_Clear(BLUE);  



    GUI_Show();



    while(1){
    	sensor();
        GUI_Show();
     }


    LCD_Exit();

}





int main(){
    Lcd_test();

}