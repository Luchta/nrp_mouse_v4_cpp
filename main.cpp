#include <iostream>
#include "MouseUI.h"
#include "RPI.h"


int main(void) {
    std::cout << "Guten Morgen Ihr Waschlappen!\n";

    CRPI maus; //Maus Objekt mit allem
    CMouseUI mouse_menu(maus); // User Interface Menu

    maus.InitRPI(); //initalize Legs
    maus.startUART();   //start communication

    maus.startThread();

    mouse_menu.mainMenu();
    /*
    while (1) {
        //do nothing
        usleep(600000);
    }
    */
    return 0;
}
