#include "cdcpd/optimizer.h"

static GRBEnv& getGRBEnv()
{
    static GRBEnv env;
    return env;
}


