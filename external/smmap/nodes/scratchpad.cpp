#include <smmap/gurobi_solvers.h>

#include <iostream>
#include <omp.h>


#define _PROGRAM_NAME "scratchpad"
#include <stdlib.h>
#include <pwd.h>
#include <stdio.h>

int main(int argc, char* argv[])
{
    (void)argc;
    (void)argv;

    #pragma omp parallel for
    for (int i = 0; i < 90; i++)
    {
//        char buf[256];
//        getlogin_r(buf, 256);
//        std::cout << buf << " ";

//        register struct passwd *pw;
//        register uid_t uid;

//        uid = geteuid();
//        pw = getpwuid(uid);
//        if (pw)
//        {
//            std::cout << pw->pw_name << " ";
//        }
//        else
//        {
//            fprintf (stderr,"%s: cannot find username for UID %u\n", _PROGRAM_NAME, (unsigned) uid);
//        }

        std::cout << smmap::minSquaredNorm(Eigen::MatrixXd::Ones(6000, 12), Eigen::VectorXd::Ones(6000), 0.1, Eigen::VectorXd::Ones(6000)).transpose() << std::endl;
    }

    return 0;
}
