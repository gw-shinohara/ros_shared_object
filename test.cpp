#include <dlfcn.h>
#include <cstdint>
#include <iostream>
#include <chrono>
#include <thread>
#include <csignal>

using namespace std;

namespace
{
    const string SO_FILE_PATH = "./libros_shared_object.so";
    volatile sig_atomic_t flag = 1;

    void handler(int signum)
    {
        flag = 0;
    }
}

int main(int argc, char* argv[])
{
    signal(SIGINT, handler);

    void* handle = dlopen(SO_FILE_PATH.c_str(), RTLD_NOW);
    cout << "dlopen: " << &handle << endl;

    intptr_t (*create)() = (intptr_t (*)())dlsym(handle, "create");
    void (*spin_some)(intptr_t) = (void (*)(intptr_t))dlsym(handle, "spin_some");
    void (*talk)(intptr_t, int) = (void (*)(intptr_t, int))dlsym(handle, "talk");
    void (*destroy)(intptr_t) = (void (*)(intptr_t))dlsym(handle, "destroy");

    intptr_t instance = (*create)();

    while (flag != 0)
    {
        (*talk)(instance, 0);
        (*spin_some)(instance);
        this_thread::sleep_for(chrono::seconds(1));
    }

    (*destroy)(instance);

    dlclose(handle);

    return 0;
}
