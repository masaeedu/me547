#include <stdlib.h>
#include <game.h>

game::game() {
    tile = 1;
}

void game::next()
{
    tile = tile + (rand() % 6) + 1;
}