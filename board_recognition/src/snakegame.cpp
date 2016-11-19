#include <stdlib.h>

class snake
{
  public:
    int tile = 1;
    void next()
    {
        tile = tile + (rand() % 6) + 1;
    }
};