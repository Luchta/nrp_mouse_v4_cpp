#include <iostream>
#include "Mouse_UI.h"
#include "Mouse_COM.h"


int main(void) {
    std::cout << "Hello Edison!\n";
    std::cout << "Starting Init"<<std::endl;
    mouse_ui mouse_menu;
    mouse_com communicator;

    communicator.startThread();

    //mousetest.init();

    mouse_menu.mainMenu();

    return 0;
}
