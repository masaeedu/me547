#include <stdlib.h>
#include <game.h>

game::game()
{
    tile = 1;
}

void game::next()
{
    tile = tile + (rand() % 6) + 1;

    if (tile == 2)
    {
        tile = 44;
    }
    else if (tile == 6)
    {
        tile = 13;
    }
    else if (tile == 9)
    {
        tile = 31;
    }
    else if (tile == 28)
    {
        tile = 84;
    }
    else if (tile == 59)
    {
        tile = 61;
    }
    else if (tile == 67)
    {
        tile = 93;
    }
    else if (tile == 70)
    {
        tile = 73;
    }
    else if (tile == 79)
    {
        tile = 100;
    }
    //Snakes
    if (tile == 16)
    {
        tile = 5;
    }
    else if (tile == 36)
    {
        tile = 21;
    }
    else if (tile == 50)
    {
        tile = 8;
    }
    else if (tile == 68)
    {
        tile = 15;
    }
    else if (tile == 63)
    {
        tile = 20;
    }
    else if (tile == 89)
    {
        tile = 45;
    }
    else if (tile == 95)
    {
        tile = 90;
    }
    else if (tile == 98)
    {
        tile = 78;
    }
}