#include <cstdio>
#include <unistd.h>
#include "PolygonStampedPublisher.h"

int main(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    int samples = 100;
    uint32_t count_stamp = 0;

    PolygonStampedPublisher *mypub = new PolygonStampedPublisher();

    if (mypub->init())
    {
        while (1)
        {
            mypub->run();
            sleep(1);
        }
    }
    else
    {
        std::cout << "fastdds init fail." << std::endl;
    }

    delete mypub;
    return 0;
}
