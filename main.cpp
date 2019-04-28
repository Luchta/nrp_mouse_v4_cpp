#include <iostream>
#include "Mouse_UI.h"
#include "RPI.h"


int main(void) {
    std::cout << "Guten Morgen Ihr Waschlappen!\n";
    CRPI maus; //Maus Objekt mit allem
    mouse_ui mouse_menu(maus); // User Interface Menu


    maus.startThread();

    mouse_menu.mainMenu();

    return 0;
}
